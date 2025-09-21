#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

// Debounce
#define DEBOUNCE_TRESHHOLD 200 // in milliseconds
#define BUTTONS_AMOUNT 6
volatile uint32_t btn_prev_tick[BUTTONS_AMOUNT]; // array for number of buttons

// Potentiometer 
volatile uint8_t pot_mode = 0;
#define POT_RESOLUTION 4095
#define ADC_BUF_LEN 1
uint16_t adc_buf[ADC_BUF_LEN];

// Telemetry
#define MAX_TEMP 100
volatile uint8_t engine_temp = 0;
volatile uint16_t rpm = 0;
volatile uint16_t speed = 0;
#define MAX_GEAR 8
#define MIN_GEAR 0
volatile uint8_t gear = 0;

// Lights objects
volatile bool turn_left = false;
volatile bool turn_right = false;
volatile uint8_t light_mode = 0;

// Fuel
volatile uint16_t fuel_level = 0;

// I2C Encoders
#define ENC1_ADDR     0x40
#define ENC2_ADDR     0x41
#define ENC_REG_HIGH  0xFE
#define ENC_REG_LOW   0xFF

HAL_StatusTypeDef ecnoder_1_state;
HAL_StatusTypeDef encoder_2_state;

typedef struct
{
    uint8_t high_byte;
    uint8_t low_byte;
    int16_t old_step;
    int16_t output;
} EncoderData;

EncoderData encoder_1 = {0};
EncoderData encoder_2 = {0};

// Function declaration 
int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range,
                      int16_t min, int16_t max);
void uart_print();
void pot_read();
void encoders_read();
void perephery_check();;

void app()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

    while(1) {
        perephery_check();
        pot_read();
        encoders_read();
        uart_print();
        // HAL_Delay(5);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        uint32_t curr_tick = HAL_GetTick();

        switch (GPIO_Pin)
        {
        case BTN_UP_Pin:
            if ((curr_tick - btn_prev_tick[0]) > DEBOUNCE_TRESHHOLD
                && gear < MAX_GEAR) {
                gear++;
                btn_prev_tick[0] = curr_tick;
            }
            break;
        
        case BTN_DOWN_Pin:
            if ((curr_tick - btn_prev_tick[1]) > DEBOUNCE_TRESHHOLD
                && gear > MIN_GEAR) {
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

void perephery_check() {
        const char *msg1 = "Encoder 1 not ready\r\n";
        const char *msg2 = "Encoder 2 not ready\r\n";

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC1_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg1, sizeof(msg1), 100);
            
            // TROW CAN ERROR
        }

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC1_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2), 100);

            // TROW CAN ERROR
        }
}

void pot_read() {
    // Potentiometer mode LED indicator
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, pot_mode);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !pot_mode);

    if (pot_mode) {
        engine_temp = adc_buf[0] * MAX_TEMP / POT_RESOLUTION;
    }
    else {
        fuel_level = adc_buf[0];
    }
}

void encoders_read() {
    HAL_I2C_Mem_Read(&hi2c3, (ENC1_ADDR << 1), ENC_REG_HIGH, I2C_MEMADD_SIZE_8BIT,
                                &encoder_1.high_byte, 1, 100);
    HAL_I2C_Mem_Read(&hi2c3, (ENC1_ADDR << 1), ENC_REG_LOW, I2C_MEMADD_SIZE_8BIT,
                                &encoder_1.low_byte, 1, 100);
    HAL_I2C_Mem_Read(&hi2c3, (ENC2_ADDR << 1), ENC_REG_HIGH, I2C_MEMADD_SIZE_8BIT,
                                &encoder_2.high_byte, 1, 100);
    HAL_I2C_Mem_Read(&hi2c3, (ENC2_ADDR << 1), ENC_REG_LOW, I2C_MEMADD_SIZE_8BIT,
                                &encoder_2.low_byte, 1, 100);

    uint16_t angle_encoder_1 = ((uint16_t)encoder_1.high_byte << 6
                                | (encoder_1.low_byte & 0x3F));
    uint16_t angle_encoder_2 = ((uint16_t)encoder_2.high_byte << 6
                                | (encoder_2.low_byte & 0x3F));

    rpm = encoder_remap(&encoder_1, angle_encoder_1, 2048, 0, 4096);
    speed = encoder_remap(&encoder_2, angle_encoder_2, 100, 0, 280);
}

int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range,
                      int16_t min, int16_t max) {
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

void uart_print() {
    char msg[128];
    
    snprintf(msg, sizeof(msg),
    "G: %u, L: %i, R: %i, LM: %i, PM: %u, FL: %hu, ET: %u, E1: %u, E2: %u.\r\n",
    gear, turn_left, turn_right, light_mode, pot_mode, fuel_level,
    engine_temp, rpm, speed);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}