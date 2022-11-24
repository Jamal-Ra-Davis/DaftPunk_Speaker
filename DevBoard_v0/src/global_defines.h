#pragma once

// Pin Defines
#define RGB_LED_EN 17
#define RGB_LED_DATA 27
#define AMP_SD_PIN 4

#define VOL_P_PIN 32
#define VOL_M_PIN 34
#define PAIR_PIN 35

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 19

#define SR_DIN_PIN 23
#define SR_CLK_PIN 18
#define SR_LAT_PIN 5

//Hardware Config
#define SR_CNT 6

// Task Priorities
#define FFT_TASK_PRIORITY 4
#define DISPLAY_TASK_PRIORITY 5
#define TIMER_THREAD_TASK_PRIORITY 1