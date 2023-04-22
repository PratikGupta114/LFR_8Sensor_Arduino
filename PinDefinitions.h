
#include <Arduino.h>
#include "Config.h"

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)
#include <avr/io.h>
#endif

#ifndef PINDEFINITIONS_H
#define PINDEFINITIONS_H

#define SENSOR_COUNT 12

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 1)

#define S1 PA0
#define S2 PA1
#define S3 PA2
#define S4 PA3
#define S5 PA4
#define S6 PA5
#define S7 PA6
#define S8 PA7

#define LEFT_MOTOR_PIN_1 PB2
#define LEFT_MOTOR_PIN_2 PB1

#define RIGHT_MOTOR_PIN_1 PD7
#define RIGHT_MOTOR_PIN_2 PD6

#define LEFT_MOTOR_PWM_PIN PB3
#define RIGHT_MOTOR_PWM_PIN PD5
#define LEFT_MOTOR_PWM_REGISTER OCR2A  // Uses Timer 2
#define RIGHT_MOTOR_PWM_REGISTER OCR1B // Uses Timer 1

#define BLUE_LED PD4
#define BUZZER PD2
#define WHITE_LED PD3

#endif

#if (BARE_METAL_GPIO_CONTROL_ENABLED == 0)

// S1 being the left most sensor pin and S8 being the rightmost sensor pin
#define S1 PA0
#define S2 PA1
#define S3 PA2
#define S4 PA3
#define S5 PA4
#define S6 PA5
#define S7 PA6
#define S8 PA7

#define LEFT_MOTOR_PIN_1 10
#define LEFT_MOTOR_PIN_2 9

#define RIGHT_MOTOR_PIN_1 7
#define RIGHT_MOTOR_PIN_2 6

#define LEFT_MOTOR_PWM_PIN 11
#define RIGHT_MOTOR_PWM_PIN 5

#define BLUE_LED 4
#define BUZZER 2
#define WHITE_LED 3

#endif // Bare metal gppio control enabled check

#endif