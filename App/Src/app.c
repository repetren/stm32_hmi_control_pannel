#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include "can.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

// Debounce
#define DEBOUNCE_TRESHHOLD 200 // in milliseconds
#define BUTTONS_AMOUNT 6
volatile uint32_t btn_prev_tick[BUTTONS_AMOUNT]; // array for number of buttons

// Potentiometer 
#define POT_RESOLUTION 4095
#define ADC_BUF_LEN 1
volatile uint8_t pot_mode = 0;
uint16_t adc_buf[ADC_BUF_LEN];

// Telemetry
#define MAX_TEMP 100
#define MAX_SPEED 280
#define MAX_GEAR 8
#define MIN_GEAR 0

typedef struct
{
    volatile uint8_t gear;
    uint8_t engine_temp;
    uint16_t rpm;
    uint16_t speed;

} TelemetryData;

// Lights objects
typedef struct
{
    volatile bool turn_left;
    volatile bool turn_right;
    volatile uint8_t light_mode;
} LightsData;

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

volatile TelemetryData telemetry = {0};
volatile LightsData lights = {0};

CAN_TxHeaderTypeDef TxHeader;

uint8_t TxData[8];
uint32_t TxMailbox;

// Function declaration 
int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range,
                      int16_t min, int16_t max);
void uart_print();
void pot_read();
void encoders_read();
void perephery_check();;

void can_send_telemetry();
void can_send_lights();
void can_send_fuel();

enum {
    PERIOD_100HZ = 100,
    PERIOD_50HZ = 200,
    PERIOD_1HZ = 10000,
};

void app()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
    HAL_CAN_Start(&hcan1);
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    HAL_TIM_Base_Start(&htim14);

    uint16_t t100hz_last = __HAL_TIM_GET_COUNTER(&htim14);
    uint16_t t50hz_last = __HAL_TIM_GET_COUNTER(&htim14);
    uint16_t t1hz_last = __HAL_TIM_GET_COUNTER(&htim14);

    while(1) {
        perephery_check();
        // uart_print();

        if (__HAL_TIM_GET_COUNTER(&htim14) - t100hz_last >= PERIOD_100HZ) {
            encoders_read();
            can_send_telemetry();
            pot_read();
            t100hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }

        if (__HAL_TIM_GET_COUNTER(&htim14) - t50hz_last >= PERIOD_50HZ) {
            can_send_lights();
            t50hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }

        if (__HAL_TIM_GET_COUNTER(&htim14) - t1hz_last >= PERIOD_1HZ) {
            can_send_fuel();
            t1hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        uint32_t curr_tick = HAL_GetTick();

        switch (GPIO_Pin)
        {
        case BTN_UP_Pin:
            if ((curr_tick - btn_prev_tick[0]) > DEBOUNCE_TRESHHOLD
                && telemetry.gear < MAX_GEAR) {
                telemetry.gear++;
                btn_prev_tick[0] = curr_tick;
            }
            break;
        
        case BTN_DOWN_Pin:
            if ((curr_tick - btn_prev_tick[1]) > DEBOUNCE_TRESHHOLD
                && telemetry.gear > MIN_GEAR) {
                telemetry.gear--;
                btn_prev_tick[1] = curr_tick;
            }
            break;

        case BTN_LEFT_Pin:
            if ((curr_tick - btn_prev_tick[2]) > DEBOUNCE_TRESHHOLD) {
                lights.turn_right = false;
                lights.turn_left = !lights.turn_left;
                btn_prev_tick[2] = curr_tick;
            }
            break;

        case BTN_RIGHT_Pin:
            if ((curr_tick - btn_prev_tick[3]) > DEBOUNCE_TRESHHOLD) {
                lights.turn_left = false;
                lights.turn_right = !lights.turn_right;
                btn_prev_tick[3] = curr_tick;
            }
            break;

        case LIGHT_MODE_Pin:
            if ((curr_tick - btn_prev_tick[4]) > DEBOUNCE_TRESHHOLD) {
                if (lights.light_mode >= 0 && lights.light_mode < 3) {
                    lights.light_mode++;
                }
                else {
                    lights.light_mode = 0;
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
        static const char msg1[] = "Encoder 1 not ready\r\n";
        static const char msg2[] = "Encoder 2 not ready\r\n";

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC1_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg1, sizeof(msg1) - 1, 200);
            
            // TROW CAN ERROR
        }

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC2_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg1) - 1, 200);

            // TROW CAN ERROR
        }
}

void pot_read() {
    // Potentiometer mode LED indicator
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, pot_mode);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !pot_mode);

    if (pot_mode) {
        telemetry.engine_temp = adc_buf[0] * MAX_TEMP / POT_RESOLUTION;
    }
    else {
        fuel_level = adc_buf[0];
    }
}

void encoders_read() {
    HAL_I2C_Mem_Read(&hi2c3, (ENC1_ADDR << 1), ENC_REG_HIGH, I2C_MEMADD_SIZE_8BIT,
                                &encoder_1.high_byte, 1, 5);
    HAL_I2C_Mem_Read(&hi2c3, (ENC1_ADDR << 1), ENC_REG_LOW, I2C_MEMADD_SIZE_8BIT,
                                &encoder_1.low_byte, 1, 5);
    HAL_I2C_Mem_Read(&hi2c3, (ENC2_ADDR << 1), ENC_REG_HIGH, I2C_MEMADD_SIZE_8BIT,
                                &encoder_2.high_byte, 1, 5);
    HAL_I2C_Mem_Read(&hi2c3, (ENC2_ADDR << 1), ENC_REG_LOW, I2C_MEMADD_SIZE_8BIT,
                                &encoder_2.low_byte, 1, 5);

    uint16_t angle_encoder_1 = ((uint16_t)encoder_1.high_byte << 6
                                | (encoder_1.low_byte & 0x3F));
    uint16_t angle_encoder_2 = ((uint16_t)encoder_2.high_byte << 6
                                | (encoder_2.low_byte & 0x3F));

    telemetry.rpm = encoder_remap(&encoder_1, angle_encoder_1, 2048, 0, 4096);
    telemetry.speed = encoder_remap(&encoder_2, angle_encoder_2, 100, 0, MAX_SPEED);
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
    
    // snprintf(msg, sizeof(msg),
    // "G: %u, L: %i, R: %i, LM: %i, PM: %u, FL: %hu, ET: %u, E1: %u, E2: %u.\r\n",
    // gear, turn_left, turn_right, light_mode, pot_mode, fuel_level,
    // engine_temp, rpm, speed);

    // snprintf(msg, sizeof(msg),
    // "Gear: %u, speed: %u , rpm: %u, engine temp: %u.\r\n",
    // gear, speed, rpm, engine_temp);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void can_send_telemetry() {
    TxHeader.DLC = 6;
    TxHeader.StdId = 0x100;

    TxData[0] = telemetry.gear;
    TxData[1] = telemetry.speed >> 8 & 0xFF; // MSB
    TxData[2] = telemetry.speed & 0xFF; // LSB
    TxData[3] = telemetry.rpm >> 8 & 0xFF;
    TxData[4] = telemetry.rpm & 0xFF;
    TxData[5] = telemetry.engine_temp;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void can_send_lights() {
    TxHeader.DLC = 1;
    TxHeader.StdId = 0x110;

    TxData[0] = (lights.turn_left & 0x01) << 7;
    TxData[0] |= (lights.turn_right & 0x01) << 6;
    TxData[0] |= (lights.light_mode & 0x03) << 4;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void can_send_fuel() {
    TxHeader.DLC = 2;
    TxHeader.StdId = 0x120;

    TxData[0] = fuel_level >> 8 & 0xFF; // MSB
    TxData[1] = fuel_level & 0xFF; // LSB

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}