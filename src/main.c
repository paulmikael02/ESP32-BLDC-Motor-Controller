#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bldc_control.h" 
#include "driver/gpio.h" // Tarvitaan Hall-testiin

static const char *TAG = "MAIN_APP";

// --- TESTIFUNKTIOT ---

void run_speed_ramp_test(void) {
    ESP_LOGI("TEST", "Starting Speed Ramp Test (0 to 250 ticks)..."); // 0-500 (max) PWM ticks
    init_bldc_pwm();
    init_hall_sensors();
    bldc_force_initial_commutation();

    while (1) {
        for (uint32_t speed = 0; speed <= 250; speed += 10) {
            set_motor_speed(speed);
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        for (uint32_t speed = 250; speed > 0; speed -= 10) {
            set_motor_speed(speed);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        set_motor_speed(0);
        vTaskDelay(pdMS_TO_TICKS(2000)); 
    }
}

void run_hall_sensor_test(void) {
    ESP_LOGI("TEST", "Hall Sensor Monitor Active. Spin the motor shaft manually!");
    init_hall_sensors();
    
    while (1) {
        uint32_t hall_val = (gpio_get_level(PIN_HALL_U) << 2) | 
                            (gpio_get_level(PIN_HALL_V) << 1) | 
                            (gpio_get_level(PIN_HALL_W));
        printf("Current Hall State: %lu\n", hall_val);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- PÄÄOHJELMA ---

void app_main(void) {
    // ---------------------------------------------------------
    // TEST MODE: Uncomment the test you want to run. 
    // Ensure that the normal operation block below is commented out.
    // ---------------------------------------------------------
    
    // run_hall_sensor_test();
    // run_speed_ramp_test();

    // ---------------------------------------------------------
    // NORMAL OPERATION: Full motor control stack
    // ---------------------------------------------------------
    ESP_LOGI(TAG, "Initializing BLDC Controller...");
    init_bldc_pwm();
    init_hall_sensors();
    init_potentiometer_adc();
    bldc_force_initial_commutation();
    bldc_start_control_task();
    
    while (1) {
        set_motor_direction(1);
        vTaskDelay(pdMS_TO_TICKS(5000));

        set_motor_direction(0);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}









