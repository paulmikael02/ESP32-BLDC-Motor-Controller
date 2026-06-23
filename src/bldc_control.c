#include "bldc_control.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"        
#include "esp_adc/adc_cali_scheme.h" 
#include "esp_timer.h"

static const char *TAG = "BLDC_HW";

// --- PRIVATE GLOBAL HANDLES (Encapsulated inside this file) ---
static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper_u = NULL, oper_v = NULL, oper_w = NULL;
static mcpwm_cmpr_handle_t cmpr_u = NULL, cmpr_v = NULL, cmpr_w = NULL;
static mcpwm_gen_handle_t gen_u_h = NULL, gen_u_l = NULL;
static mcpwm_gen_handle_t gen_v_h = NULL, gen_v_l = NULL;
static mcpwm_gen_handle_t gen_w_h = NULL, gen_w_l = NULL;
static adc_oneshot_unit_handle_t adc1_handle = NULL; 
static adc_cali_handle_t cali_handle = NULL;
static volatile uint64_t bldc_last_hall_time = 0;
static volatile uint32_t bldc_rev_period_us = 0;

volatile uint32_t current_duty = 0;
volatile uint32_t latest_hall_val = 0; // Stores the latest Hall state safely for telemetry

// Direction control: 1 = Forward, 0 = Reverse
volatile uint8_t motor_direction = 1;
volatile uint8_t motor_direction_changing = 0;

// --- PRIVATE HARDWARE COMMUTATION LOGIC ---
// Placed in IRAM for maximum execution speed inside the interrupt context
static inline void IRAM_ATTR update_bldc_commutation(uint32_t hall_val) {
    // Forward sequence verified with the Vevor BLDC motor
    const uint8_t seq_forward[6] = {4, 6, 2, 3, 1, 5};
    // Reverse sequence: exactly inverted order
    const uint8_t seq_reverse[6] = {3, 4, 5, 2, 6, 1};

    const uint8_t *seq = (motor_direction == 1) ? seq_forward : seq_reverse;

    int index = -1;
    for (int i = 0; i < 6; i++) {
        if (seq[i] == hall_val) {
            index = i;
            break;
        }
    }

    if (index != -1) {
        int shifted_index = (index + 1) % 6; // Always advance +1 within the selected sequence table
        hall_val = seq[shifted_index];
    }

    // Apply the 6-step commutation states to the MCPWM generators
    switch(hall_val) {
        case 5: 
            mcpwm_generator_set_force_level(gen_u_h, 0, true);  mcpwm_generator_set_force_level(gen_u_l, 1, true);
            mcpwm_generator_set_force_level(gen_v_h, 0, true);  mcpwm_generator_set_force_level(gen_v_l, 0, true);
            mcpwm_generator_set_force_level(gen_w_h, -1, true); mcpwm_generator_set_force_level(gen_w_l, -1, true);
            break;
        case 1: 
            mcpwm_generator_set_force_level(gen_u_h, 0, true);  mcpwm_generator_set_force_level(gen_u_l, 0, true);
            mcpwm_generator_set_force_level(gen_v_h, 0, true);  mcpwm_generator_set_force_level(gen_v_l, 1, true);
            mcpwm_generator_set_force_level(gen_w_h, -1, true); mcpwm_generator_set_force_level(gen_w_l, -1, true);
            break;
        case 3: 
            mcpwm_generator_set_force_level(gen_u_h, -1, true); mcpwm_generator_set_force_level(gen_u_l, -1, true);
            mcpwm_generator_set_force_level(gen_v_h, 0, true);  mcpwm_generator_set_force_level(gen_v_l, 1, true);
            mcpwm_generator_set_force_level(gen_w_h, 0, true);  mcpwm_generator_set_force_level(gen_w_l, 0, true);
            break;
        case 2: 
            mcpwm_generator_set_force_level(gen_u_h, -1, true); mcpwm_generator_set_force_level(gen_u_l, -1, true);
            mcpwm_generator_set_force_level(gen_v_h, 0, true);  mcpwm_generator_set_force_level(gen_v_l, 0, true);
            mcpwm_generator_set_force_level(gen_w_h, 0, true);  mcpwm_generator_set_force_level(gen_w_l, 1, true);
            break;
        case 6: 
            mcpwm_generator_set_force_level(gen_u_h, 0, true);  mcpwm_generator_set_force_level(gen_u_l, 0, true);
            mcpwm_generator_set_force_level(gen_v_h, -1, true); mcpwm_generator_set_force_level(gen_v_l, -1, true);
            mcpwm_generator_set_force_level(gen_w_h, 0, true);  mcpwm_generator_set_force_level(gen_w_l, 1, true);
            break;
        case 4: 
            mcpwm_generator_set_force_level(gen_u_h, 0, true);  mcpwm_generator_set_force_level(gen_u_l, 1, true);
            mcpwm_generator_set_force_level(gen_v_h, -1, true); mcpwm_generator_set_force_level(gen_v_l, -1, true);
            mcpwm_generator_set_force_level(gen_w_h, 0, true);  mcpwm_generator_set_force_level(gen_w_l, 0, true);
            break;
        default:
            // Safe default state: shut down all phases to prevent short circuits
            mcpwm_generator_set_force_level(gen_u_h, 0, true); mcpwm_generator_set_force_level(gen_u_l, 0, true);
            mcpwm_generator_set_force_level(gen_v_h, 0, true); mcpwm_generator_set_force_level(gen_v_l, 0, true);
            mcpwm_generator_set_force_level(gen_w_h, 0, true); mcpwm_generator_set_force_level(gen_w_l, 0, true);
            break;
    }
}

// --- PRIVATE HALL SENSOR INTERRUPT CALLBACK ---
static bool IRAM_ATTR hall_sensor_callback(mcpwm_cap_channel_handle_t cap_channel, const mcpwm_capture_event_data_t *edata, void *user_ctx) {
    // Read current Hall sensor states and build a 3-bit value (U = MSB, W = LSB)
    uint32_t hall_val = (gpio_get_level(PIN_HALL_U) << 2) |
                        (gpio_get_level(PIN_HALL_V) << 1) |
                        (gpio_get_level(PIN_HALL_W));
    latest_hall_val = hall_val;
    update_bldc_commutation(hall_val);

    // Measure time period between consecutive U phase pulses for RPM tracking
    // Checks if it is the Hall U channel (context == 1) and a positive rising edge
    if ((int)user_ctx == 1 && edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        uint64_t now = esp_timer_get_time();
        bldc_rev_period_us = now - bldc_last_hall_time;
        bldc_last_hall_time = now;
    }

    return false; // Return false as we do not need a FreeRTOS task yield here
}

// --- POTENTIOMETER READING AND SPEED CONTROL TASK ---
static void potentiometer_task(void *pvParameters) {
    uint32_t adc_sum = 0;
    const int SAMPLES = 16; 
    static float smoothed_rpm = 0.0f;
    const float FILTER_ALPHA = 0.10f; // Low-pass filter coefficient for RPM smoothing

    while (1) {
        // Oversampling the ADC input to filter out high-frequency noise
        adc_sum = 0;
        for (int i = 0; i < SAMPLES; i++) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_POT_CHANNEL, &raw));
            adc_sum += raw;
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }
        
        int raw_avg = adc_sum / SAMPLES;
        int voltage_mv = 0;

        // Convert raw ADC value to factory-calibrated voltage in millivolts (mV)
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw_avg, &voltage_mv));

        // Scale voltage to PWM ticks: 0 - 3300 mV maps to 0 - 500 ticks
        uint32_t target_duty = (voltage_mv * PWM_PERIOD) / 3300;

        // Add a small deadband at the bottom to stop the motor from whining at near-zero throttle
        if (voltage_mv < 15) {
            target_duty = 0;
        }

        if (target_duty > PWM_PERIOD) {
            target_duty = PWM_PERIOD;
        }
        
        // Skip updating speed if a hardware direction swap is currently active
        if (motor_direction_changing) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        set_motor_speed(target_duty);

        // --- RPM CALCULATION ---
        float raw_rpm = 0.0f;
        uint32_t local_period = bldc_rev_period_us;
        uint64_t current_time = esp_timer_get_time();
        
        const float POLE_PAIRS = 1.0f; 

        // Timeout logic: If the motor has been stopped for > 200ms or period is invalid, force RPM to 0
        if ((current_time - bldc_last_hall_time > 200000) || local_period < 1000) {
            raw_rpm = 0.0f;
        } else {
            // Protected division: calculate electrical RPM based on hardware timer ticks
            raw_rpm = 60000000.0f / (POLE_PAIRS * (float)local_period);    
            
            // Invert the RPM value mathematically if running in reverse
            if (motor_direction == 0) {
                raw_rpm = -raw_rpm;
            }
        }

        // Apply exponential moving average filter to smooth out RPM fluctuations
        if (raw_rpm == 0.0f) {
            smoothed_rpm = 0.0f;
        } else {
            smoothed_rpm = (smoothed_rpm * (1.0f - FILTER_ALPHA)) + (raw_rpm * FILTER_ALPHA);
        }

        // Print values formatted for serial monitoring or Teleplot visualization
        printf(">Motor_Duty_Ticks:%lu\n", target_duty);
        printf(">Hall_State:%lu\n", latest_hall_val);
        printf(">Motor_RPM:%.1f\n", smoothed_rpm); 
        printf(">Motor_Direction:%d\n", motor_direction); 
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

// --- PUBLIC FUNCTION IMPLEMENTATIONS ---

void init_bldc_pwm(void) {
    ESP_LOGI(TAG, "Initializing MCPWM Timers...");
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RES_HZ,
        .period_ticks = PWM_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
 
    mcpwm_operator_config_t oper_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper_u));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper_v));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper_w));
   
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_u, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_v, timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_w, timer));
 
    mcpwm_comparator_config_t cmpr_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_u, &cmpr_config, &cmpr_u));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_v, &cmpr_config, &cmpr_v));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_w, &cmpr_config, &cmpr_w));
 
    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = PIN_U_H; ESP_ERROR_CHECK(mcpwm_new_generator(oper_u, &gen_config, &gen_u_h));
    gen_config.gen_gpio_num = PIN_U_L; ESP_ERROR_CHECK(mcpwm_new_generator(oper_u, &gen_config, &gen_u_l));
    gen_config.gen_gpio_num = PIN_V_H; ESP_ERROR_CHECK(mcpwm_new_generator(oper_v, &gen_config, &gen_v_h));
    gen_config.gen_gpio_num = PIN_V_L; ESP_ERROR_CHECK(mcpwm_new_generator(oper_v, &gen_config, &gen_v_l));
    gen_config.gen_gpio_num = PIN_W_H; ESP_ERROR_CHECK(mcpwm_new_generator(oper_w, &gen_config, &gen_w_h));
    gen_config.gen_gpio_num = PIN_W_L; ESP_ERROR_CHECK(mcpwm_new_generator(oper_w, &gen_config, &gen_w_l));
 
    mcpwm_generator_set_action_on_timer_event(gen_u_h, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_u_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_u, MCPWM_GEN_ACTION_LOW));
   
    mcpwm_generator_set_action_on_timer_event(gen_v_h, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_v_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_v, MCPWM_GEN_ACTION_LOW));
   
    mcpwm_generator_set_action_on_timer_event(gen_w_h, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_w_h, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_w, MCPWM_GEN_ACTION_LOW));
 
    // Configure hardware dead-time to prevent power bridge shoot-through short circuits
    mcpwm_dead_time_config_t dt_config = {
        .posedge_delay_ticks = DEAD_TIME_TICKS,
        .negedge_delay_ticks = 0,
    };
   
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_u_h, gen_u_h, &dt_config));
    dt_config.posedge_delay_ticks = 0; dt_config.negedge_delay_ticks = DEAD_TIME_TICKS; dt_config.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_u_h, gen_u_l, &dt_config));
 
    dt_config.flags.invert_output = false; dt_config.posedge_delay_ticks = DEAD_TIME_TICKS; dt_config.negedge_delay_ticks = 0;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_v_h, gen_v_h, &dt_config));
    dt_config.posedge_delay_ticks = 0; dt_config.negedge_delay_ticks = DEAD_TIME_TICKS; dt_config.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_v_h, gen_v_l, &dt_config));
 
    dt_config.flags.invert_output = false; dt_config.posedge_delay_ticks = DEAD_TIME_TICKS; dt_config.negedge_delay_ticks = 0;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_w_h, gen_w_h, &dt_config));
    dt_config.posedge_delay_ticks = 0; dt_config.negedge_delay_ticks = DEAD_TIME_TICKS; dt_config.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_w_h, gen_w_l, &dt_config));
 
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void init_hall_sensors(void) {
    ESP_LOGI(TAG, "Initializing Hall Sensor Capture Subsystem...");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_config, &cap_timer));
 
    mcpwm_cap_channel_handle_t cap_ch_u, cap_ch_v, cap_ch_w;
    mcpwm_capture_channel_config_t cap_ch_config = {};
    cap_ch_config.prescale = 1;
    cap_ch_config.flags.pos_edge = true;
    cap_ch_config.flags.neg_edge = true;
    cap_ch_config.flags.pull_up = false;
   
    cap_ch_config.gpio_num = PIN_HALL_U; ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_config, &cap_ch_u));
    cap_ch_config.gpio_num = PIN_HALL_V; ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_config, &cap_ch_v));
    cap_ch_config.gpio_num = PIN_HALL_W; ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_config, &cap_ch_w));
 
    mcpwm_capture_event_callbacks_t cbs = { .on_cap = hall_sensor_callback };
    // Pass context IDs (1, 2, 3) so the ISR can accurately identify the active channel
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_ch_u, &cbs, (void*)1));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_ch_v, &cbs, (void*)2));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_ch_w, &cbs, (void*)3));
 
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_ch_u));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_ch_v));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_ch_w));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
}

void init_potentiometer_adc(void) {
    ESP_LOGI(TAG, "Initializing Potentiometer Analog Unit with Factory Calibration...");
    
    // 1. Initialize standard ADC oneshot unit
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // 2. Configure ADC channel with 12 dB attenuation (required for 3.3 V full-scale input)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
        .atten = ADC_ATTEN_DB_12,         
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_POT_CHANNEL, &config));

    // 3. Create a Curve Fitting calibration scheme on the fly to bypass non-linearity
    adc_cali_curve_fitting_config_t cali_config = { 
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
}

void bldc_force_initial_commutation(void) {
    // Read startup position from Hall lines to bootstrap initial stage before interrupts trigger
    uint32_t initial_hall = (gpio_get_level(PIN_HALL_U) << 2) | 
                            (gpio_get_level(PIN_HALL_V) << 1) | 
                            (gpio_get_level(PIN_HALL_W));
    update_bldc_commutation(initial_hall);
}

void bldc_start_control_task(void) {
    ESP_LOGI(TAG, "Spawning potentiometer thread loop...");
    xTaskCreate(potentiometer_task, "pot_speed_ctrl", 3072, NULL, 5, NULL);
}

void set_motor_speed(uint32_t duty_ticks) {
    if (duty_ticks > PWM_PERIOD) duty_ticks = PWM_PERIOD;
    mcpwm_comparator_set_compare_value(cmpr_u, duty_ticks);
    mcpwm_comparator_set_compare_value(cmpr_v, duty_ticks);
    mcpwm_comparator_set_compare_value(cmpr_w, duty_ticks);
}

void set_motor_direction(uint8_t direction) {
    if (direction > 1) direction = 1;
    
    motor_direction_changing = 1;
    set_motor_speed(0);             // Safe braking: clamp speed to zero before changing direction
    vTaskDelay(pdMS_TO_TICKS(500)); // Allow rotor inertia to drop during a 500ms delay
    
    motor_direction = direction;
    ESP_LOGI("DIR", "Direction changed to: %d", motor_direction); 
    bldc_force_initial_commutation(); // Re-bootstrap commutation state for the new direction
    
    motor_direction_changing = 0;
}