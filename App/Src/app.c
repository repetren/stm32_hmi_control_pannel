#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

uint32_t press_count = 0;
uint8_t gear = 0;

#define DEBOUNCE_TRESHHOLD 200 // in milliseconds
#define BUTTONS_AMOUNT 6
volatile uint32_t curr_tick = 0;
volatile uint32_t btn_prev_tick[BUTTONS_AMOUNT]; // array for number of buttons
volatile uint8_t pot_mode = 0;

uint8_t encoder_2_high = 0;
uint8_t encoder_2_low = 0;

volatile bool turn_left = false;
volatile bool turn_right = false;
volatile uint8_t light_mode = 0;
volatile uint16_t fuel_level = 0;
volatile uint32_t rpm = 0;
volatile uint8_t engine_temp = 0;
#define MAX_TEMP 100
#define POT_RESOLUTION 4096;

char msg[128];

typedef struct
{
    uint8_t high_byte;
    uint8_t low_byte;

    int16_t old_step;
    int16_t output;
} EncoderData;

EncoderData encoder_1 = {0};
EncoderData encoder_2 = {0};

int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range, int16_t min, int16_t max);

void app()
{
    HAL_ADC_Start(&hadc1);

    while(1) {

        // Potentiometer mode LED indicator
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, pot_mode);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !pot_mode);

        // Potentiomentr read
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        if (pot_mode) {
            engine_temp = HAL_ADC_GetValue(&hadc1) * MAX_TEMP / 4096;
        }
        else {
            fuel_level = HAL_ADC_GetValue(&hadc1);
        }

        // Work with encoders
        HAL_I2C_Mem_Read(&hi2c3, (0x40 << 1), 0xFE, I2C_MEMADD_SIZE_8BIT, &encoder_1.high_byte, 1, 100);
        HAL_I2C_Mem_Read(&hi2c3, (0x40 << 1), 0xFF, I2C_MEMADD_SIZE_8BIT, &encoder_1.low_byte, 1, 100);
        HAL_I2C_Mem_Read(&hi2c3, (0x41 << 1), 0xFE, I2C_MEMADD_SIZE_8BIT, &encoder_2_high, 1, 100);
        HAL_I2C_Mem_Read(&hi2c3, (0x41 << 1), 0xFF, I2C_MEMADD_SIZE_8BIT, &encoder_2_low, 1, 100);

	    uint16_t angle_encoder_1 = ((uint16_t)encoder_1.high_byte << 6 | (encoder_1.low_byte & 0x3));
	    uint16_t angle_encoder_2 = ((uint16_t)encoder_2_high << 6 | (encoder_2_low & 0x3));

        int16_t remaped_encoder_1 = encoder_remap(&encoder_1, angle_encoder_1, 2048, 0, 4096);
        int16_t remaped_encoder_2 = encoder_remap(&encoder_2, angle_encoder_2, 100, 0, 280);

        sprintf(msg, "G: %i, L: %d, R: %d, LM: %i, PM: %i, FL: %u, ET: %u E1: %u, E2: %u.\r\n",
            gear, turn_left, turn_right, light_mode, pot_mode, fuel_level, engine_temp, remaped_encoder_1, remaped_encoder_2);

        // sprintf(msg, "rpm: %i, speed: %i.\r\n", remaped_encoder_1, remaped_encoder_2);

        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(50);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        curr_tick = HAL_GetTick();

        switch (GPIO_Pin)
        {
        case BTN_UP_Pin:
            if ((curr_tick - btn_prev_tick[0]) > DEBOUNCE_TRESHHOLD  && gear < 8) {
                gear++;
                btn_prev_tick[0] = curr_tick;
            }
            break;
        
        case BTN_DOWN_Pin:
            if ((curr_tick - btn_prev_tick[1]) > DEBOUNCE_TRESHHOLD && gear > 0) {
                gear--;
                btn_prev_tick[1] = curr_tick;
            }
            break;

        case BTN_LEFT_Pin:
            if ((curr_tick - btn_prev_tick[2]) > DEBOUNCE_TRESHHOLD) {
                turn_right = false;
                turn_left = !turn_left;
                btn_prev_tick[2] = curr_tick;
            }
            break;

        case BTN_RIGHT_Pin:
            if ((curr_tick - btn_prev_tick[3]) > DEBOUNCE_TRESHHOLD) {
                turn_left = false;
                turn_right = !turn_right;
                btn_prev_tick[3] = curr_tick;
            }
            break;

        case LIGHT_MODE_Pin:
            if ((curr_tick - btn_prev_tick[4]) > DEBOUNCE_TRESHHOLD) {
                if (light_mode >= 0 && light_mode < 3) {
                    light_mode++;
                }
                else {
                    light_mode = 0;
                }
                btn_prev_tick[4] = curr_tick;
            }
            break;

        case BTN_POT_MODE_Pin:
            if ((curr_tick - btn_prev_tick[5] > DEBOUNCE_TRESHHOLD)) {
                pot_mode = !pot_mode;
                btn_prev_tick[5] = curr_tick;
            }
            break;

        
        default:
            break;
        }
    }

int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range, int16_t min, int16_t max) {
    uint16_t step = (uint32_t)encoder_angle * range / 16384;

    int16_t diff = enc->old_step - step;

    if (step != enc->old_step && abs(diff) < range / 2) {
        enc->output += diff;
    }

    enc->old_step = step;

    if (enc->output > max) enc->output = max;
    else if (enc->output < min) enc->output = min;

    return enc->output;
}