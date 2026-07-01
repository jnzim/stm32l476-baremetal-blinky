#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "clock.h"
#include "spi.h"
#include "tim1.h"
#include "drive.h"
#include "ringBuffer.h"
#include "loops.h"
#include "plant.h"
#include "protocol.h"
#include "config.h"
#include "encoder.h"
#include "drv8353.h"
#include "board_f411.h"
#include "pwm.h"
#include "current_feedback.h"




static float sysid_t     = 0.0f;
static float sysid_phase = 0.0f;
static float sysid_f     = SYSID_F_START;


typedef enum
{
    FOC_STAGE_ALIGN = 0,
    FOC_STAGE_RUN,
    FOC_STAGE_SYSID,
    FOC_STAGE_IDLE
} FocStage;


/* =============================================================================
 * FOC commutation state
 * =============================================================================*/
static FocStage foc_stage      = FOC_STAGE_ALIGN;
static uint32_t foc_align_tick = 0;
static int32_t  encoder_offset = 0;


/* =============================================================================
 * Global system state
 * =============================================================================*/
volatile uint32_t tick_ms = 0;
volatile bool     system_initialized = false;
volatile uint32_t tim1_isr_count = 0;


/* =============================================================================
 * Current feedback debug values
 *
 * ia/ib/ic/i_d/i_q are updated from ADC/DMA when a fresh sample is available.
 *
 * i_d_plot / i_q_plot are low-pass filtered versions for telemetry display only.
 * Do not use the plot-filtered values for control.
 * =============================================================================*/
static float ia_meas  = 0.0f;
static float ib_meas  = 0.0f;
static float ic_meas  = 0.0f;
static float i_d_meas = 0.0f;
static float i_q_meas = 0.0f;

static float i_d_plot = 0.0f;
static float i_q_plot = 0.0f;


/* =============================================================================
 * Applied voltage debug values
 *
 * These reflect the actual voltage-mode FOC command currently sent to PWM.
 * Telemetry should use these during bring-up, not the old simulated v_q_cmd.
 * =============================================================================*/
static float foc_vq_applied = 0.0f;
static float foc_vd_applied = 0.0f;



/* =============================================================================
 * foc_theta_from_encoder
 *
 * Electrical angle from encoder, referenced to captured alignment offset.
 *
 *   theta_mech = encoder counts -> mechanical radians
 *   theta_elec = pole_pairs * theta_mech + offset
 *
 * encoder_offset is captured after d-axis alignment.
 * =============================================================================*/
static float foc_theta_from_encoder(void)
{
    int32_t raw = encoder_get_position() - encoder_offset;

    float theta_mech = (float)raw * (FOC_TWO_PI / ENCODER_CPR);
    float theta_elec = ENC_DIR * theta_mech * (float)MOTOR_POLE_PAIRS;

    theta_elec = fmodf(theta_elec, FOC_TWO_PI);

    if (theta_elec < 0.0f)
        theta_elec += FOC_TWO_PI;

    return theta_elec;
}

/* =============================================================================
 * run_foc_commutation
 *
 * Temporary encoder-angle voltage-mode commutation test.
 *
 * This is NOT closed current-loop FOC yet.
 *
 * Stage 1:
 *   Apply d-axis voltage at electrical angle zero.
 *   Rotor pulls into alignment.
 *
 * Stage 2:
 *   Capture encoder position as electrical zero.
 *   Use encoder-derived electrical angle and apply q-axis voltage.
 *
 * =============================================================================*/
static void run_foc_commutation(void)
{
    switch (foc_stage)
    {
        case FOC_STAGE_ALIGN:
        {
            /*
             * Force d-axis at electrical zero.
             *
             * v_q = 0
             * v_d = V_ALIGN
             * theta = 0
             */
            foc_vq_applied = 0.0f;
            foc_vd_applied = 3.0f;
            

            pwm_apply_vq(foc_vq_applied, foc_vd_applied, 0.0f);

            if (++foc_align_tick >= ALIGN_TICKS)
            {
                encoder_offset = encoder_get_position();
            
            #if RUN_MODE == RUN_MODE_SYSID
                foc_stage = FOC_STAGE_SYSID;
            #else
                foc_stage = FOC_STAGE_RUN;
            #endif
            }
        }
        break;

        case FOC_STAGE_RUN:
        {
           
             //Use encoder-derived electrical angle.
            float theta_elec = foc_theta_from_encoder();

            foc_vq_applied = V_RUN;
            foc_vd_applied = 0.0f;

            pwm_apply_vq(foc_vq_applied, foc_vd_applied, theta_elec);
        }
        break;

        case FOC_STAGE_SYSID:
        {
            // Log-linear frequency sweep
            sysid_f = SYSID_F_START * powf(SYSID_F_END / SYSID_F_START,
                                            sysid_t / SYSID_DURATION);
            
            // Integrate phase to avoid discontinuities
            sysid_phase += 2.0f * M_PI * sysid_f * SYSID_DT;
            if (sysid_phase > 2.0f * M_PI) sysid_phase -= 2.0f * M_PI;
            
            foc_vd_applied = SYSID_AMPLITUDE * sinf(sysid_phase);
            foc_vq_applied = 0.0f;
            pwm_apply_vq(foc_vq_applied, foc_vd_applied, 0.0f);
            
            // Get current feedback
            current_feedback_get_dq(0.0f, &i_d_meas, &i_q_meas);
            
            sysid_t += SYSID_DT;
            if (sysid_t >= SYSID_DURATION)
            {
                foc_stage = FOC_STAGE_IDLE;
            }
        }
        break;

        default:
        {
            foc_stage = FOC_STAGE_ALIGN;
            foc_align_tick = 0;
            encoder_offset = 0;
        }
        break;
    }
}


/* =============================================================================
 * foc_commutation_reset
 *
 * Optional reset for re-enable / fault recovery.
 * =============================================================================*/
void foc_commutation_reset(void)
{
    foc_stage      = FOC_STAGE_ALIGN;
    foc_align_tick = 0;
    encoder_offset = 0;

    foc_vq_applied = 0.0f;
    foc_vd_applied = 0.0f;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    tim1_isr_count++;

    TIM1->SR = ~TIM_SR_UIF;

    if (!system_initialized)
        return;

    encoder_update(tick_ms);

    run_foc_commutation();


    volatile float theta;
    if (foc_stage == FOC_STAGE_SYSID)
    {
        theta = 0.0f;
    }
    else
    {
        theta = foc_theta_from_encoder();
    }

    uint16_t flags = 0;
    if (current_feedback_sample_valid())
    {
        current_feedback_get_phase_amps(&ia_meas, &ib_meas, &ic_meas);
        current_feedback_get_dq(theta, &i_d_meas, &i_q_meas);
        i_q_plot += 0.02f * (i_q_meas - i_q_plot);
        i_d_plot += 0.02f * (i_d_meas - i_d_plot);
        flags |= 0x0001u;
    }

    int32_t enc_pos = encoder_get_position();

    spi_sysid_update_latest(
        (int16_t)(enc_pos >> 16),              /* enc_hi */
        (int16_t)(enc_pos & 0xFFFF),           /* enc_lo */
        (int16_t)sysid_f,                      /* sysid_f */
        (int16_t)(i_d_meas * 1000.0f),         /* id_mA   */
        (int16_t)(i_q_meas * 1000.0f),         /* iq_mA   */
        (int16_t)(foc_vd_applied * 1000.0f),   /* vd_mV   */
        (int16_t)(foc_vq_applied * 1000.0f),   /* vq_mV   */
        (int16_t)(theta * 1000.0f),            /* theta   */
        (int16_t)(ia_meas * 1000.0f),          /* adc_a — repurpose for ia */
        (int16_t)(ib_meas * 1000.0f),          /* adc_b — repurpose for ib */
        0,
        flags
    );
}


/* =============================================================================
 * main
 * =============================================================================*/
int main(void)
{
    
    system_initialized = false;

     //  Enable FPU coprocessor.
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));

    /*
     * Core board/peripheral initialization.
     */
    clock_init();
    encoder_init();
    drive_init();
    spi_init();
    ring_init();

    /* Wait for Pi FIRE_SYSID trigger on PC3 (Pi GPIO16) */
    GPIOC->MODER &= ~(3u << (3u * 2u));
    GPIOC->PUPDR &= ~(3u << (3u * 2u));
    GPIOC->PUPDR |=  (1u << (3u * 2u));   /* pull-up */

    while (!(GPIOC->IDR & (1u << 3u))) {}  /* wait for high */
    while (GPIOC->IDR & (1u << 3u)) {}     /* wait for low */



    /*
     * DRV8353 initialization.
     */
    drv8353_init();

    bool cfg_ok = drv8353_configure();

    /*
     * Debug readback.
     *
     * Expected:
     *   csa_ctrl = 0x0283
     */
    volatile uint16_t csa_ctrl = drv8353_read_reg(DRV8353_REG_CSA_CONTROL);
    csa_ctrl = drv8353_read_reg(DRV8353_REG_CSA_CONTROL);
   
     
    /*
     * Configure and start TIM1.
     *
     * PWM bridge outputs are still disabled because pwm_init()
     * leaves TIM1 BDTR/MOE cleared.
     *
     * TIM1 is running, so TIM1_CC4 can trigger ADC injected conversions.
     */
    pwm_init();


    /*
     * Configure ADC current feedback.
     *
     * ADC waits for TIM1_CC4.
     */
    current_feedback_init();

    /*
     * Enable DRV before current calibration so current-sense amplifiers are alive.
     *
     * PWM bridge outputs are still disabled here, so the motor should not be driven.
     */
    drv_enable_high();

    /*
     * Calibrate current feedback zero offsets with PWM bridge outputs disabled.
     *
     * This requires TIM1 to already be running because calibration waits for
     * TIM1_CC4-triggered ADC injected samples.
     */
    current_feedback_calibrate();


     pwm_enable();

    

    /* Start 1 kHz SysTick only after PWM/DRV/current feedback are ready.*/
    SysTick_Config(SystemCoreClock / 1000u);

    // Allow interrupt handlers to start doing work.

    system_initialized = true;

    while (1)
    {
        __WFI();
    }
}