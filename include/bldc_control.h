#ifndef BLDC_CONTROL_H
#define BLDC_CONTROL_H

#include <stdint.h>

// --- PIN CONFIGURATIONS ---
#define PIN_U_H 11
#define PIN_U_L 10
#define PIN_V_H 19
#define PIN_V_L 7
#define PIN_W_H 0
#define PIN_W_L 6
 
#define PIN_HALL_U 5
#define PIN_HALL_V 4
#define PIN_HALL_W 18

#define PIN_POTENTIOMETER 3
#define ADC_POT_CHANNEL ADC_CHANNEL_3

// --- SYSTEM PARAMETERS ---
#define PWM_RES_HZ 10000000 // 10 MHz clock for MCPWM module
#define PWM_PERIOD 500      // 20 kHz PWM frequency (10MHz / 500)
#define DEAD_TIME_TICKS 10  // 1 microsecond deadtime delay



// --- PUBLIC FUNCTION DECLARATIONS ---
void init_bldc_pwm(void);
void init_hall_sensors(void);
void init_potentiometer_adc(void);
void bldc_force_initial_commutation(void);
void bldc_start_control_task(void);
void set_motor_speed(uint32_t duty_ticks);
void set_motor_direction(uint8_t direction);

#endif // BLDC_CONTROL_H