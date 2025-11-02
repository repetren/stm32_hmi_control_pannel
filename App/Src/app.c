#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "app.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include "can.h"
#include "tim.h"
#include "usart.h"

#include "constants.h"
#include "encoder_utils.h"
#include "send_CAN_utils.h"

// Debounce
typedef enum {
    BTN_UP_ID,
    BTN_DOWN_ID,
    BTN_LEFT_ID,
    BTN_RIGHT_ID,
    LIGHT_MODE_ID,
    BTN_POT_MODE_ID,
    BUTTONS_NUM,
} ButtonId;

volatile uint32_t btn_prev_tick[BUTTONS_NUM] = {0}; // array for buttons timestemps

// Potentiometer 
uint16_t pot_value = 0;
volatile uint8_t pot_mode = 0;
volatile uint16_t adc_buf[ADC_BUF_LEN] = {0};

volatile TelemetryData telemetry = {0};
volatile LightsData lights = {0};

static Notification low_fuel_alert = {
    VEH_INFO,
    LOW_FUEL_NOTIF_CODE,
    NOTI_CLEAR,
    0};
static Notification engine_temp_alert = {
    VEH_INFO,
    HIGH_ENGINE_TEMP_NOTIF_CODE,
    NOTI_CLEAR,
    0};

VehicleInfo info = {
    .odo = DEMO_ODO,
    .trip_a = TRIP_A_DEMO,
    .fuel_level = 0,
    .fuel_range = 0,
    .low_fuel_lamp = false,
};

EncoderData encoder_1 = {0};
EncoderData encoder_2 = {0};

// Function declarations

// void uart_print();
void pot_read();
void encoders_read();
void perephery_check();

void vechicle_info();
float fuel_level_to_liters(uint16_t fuel_level);

enum Period {
    PERIOD_100HZ = TIMER14_PER_SEC / 100,
    PERIOD_50HZ = TIMER14_PER_SEC / 50,
    PERIOD_25HZ = TIMER14_PER_SEC / 25,
};

void app()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
    HAL_CAN_Start(&hcan1);

    HAL_TIM_Base_Start(&htim14);
    HAL_TIM_Base_Start(&htim13);

    uint16_t t100hz_last = __HAL_TIM_GET_COUNTER(&htim14);
    uint16_t t50hz_last = __HAL_TIM_GET_COUNTER(&htim14);
    uint16_t t1hz_last = __HAL_TIM_GET_COUNTER(&htim14);

    while(true) {
        // uart_print();

        vechicle_info();

        if (__HAL_TIM_GET_COUNTER(&htim14) - t100hz_last >= PERIOD_100HZ) {
            encoders_read();
            send_CAN_telemetry(&telemetry);
            pot_read();
            t100hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }

        if (__HAL_TIM_GET_COUNTER(&htim14) - t50hz_last >= PERIOD_50HZ) {
            send_CAN_light(&lights);
            t50hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }

        if (__HAL_TIM_GET_COUNTER(&htim14) - t1hz_last >= PERIOD_25HZ) {
            perephery_check();
            send_CAN_fuel(&info);
            send_CAN_veh_info(&info);
            t1hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        uint32_t curr_tick = HAL_GetTick();

        switch (GPIO_Pin)
        {
        case BTN_UP_Pin:
            if ((curr_tick - btn_prev_tick[BTN_UP_ID]) > DEBOUNCE_TRESHHOLD
                && telemetry.gear < MAX_GEAR) {
                telemetry.gear++;
                btn_prev_tick[BTN_UP_ID] = curr_tick;
            }
            break;
        
        case BTN_DOWN_Pin:
            if ((curr_tick - btn_prev_tick[BTN_DOWN_ID]) > DEBOUNCE_TRESHHOLD
                && telemetry.gear > MIN_GEAR) {
                telemetry.gear--;
                btn_prev_tick[BTN_DOWN_ID] = curr_tick;
            }
            break;

        case BTN_LEFT_Pin:
            if ((curr_tick - btn_prev_tick[BTN_LEFT_ID]) > DEBOUNCE_TRESHHOLD) {
                lights.turn_right = false;
                lights.turn_left = !lights.turn_left;
                btn_prev_tick[BTN_LEFT_ID] = curr_tick;
            }
            break;

        case BTN_RIGHT_Pin:
            if ((curr_tick - btn_prev_tick[BTN_RIGHT_ID]) > DEBOUNCE_TRESHHOLD) {
                lights.turn_left = false;
                lights.turn_right = !lights.turn_right;
                btn_prev_tick[BTN_RIGHT_ID] = curr_tick;
            }
            break;

        case LIGHT_MODE_Pin:
            if ((curr_tick - btn_prev_tick[LIGHT_MODE_ID]) > DEBOUNCE_TRESHHOLD) {
                if (lights.light_mode >= 0 && lights.light_mode < 3) {
                    lights.light_mode++;
                }
                else {
                    lights.light_mode = 0;
                }
                btn_prev_tick[LIGHT_MODE_ID] = curr_tick;
            }
            break;

        case BTN_POT_MODE_Pin:
            if ((curr_tick - btn_prev_tick[BTN_POT_MODE_ID] > DEBOUNCE_TRESHHOLD)) {
                pot_mode = !pot_mode;
                btn_prev_tick[BTN_POT_MODE_ID] = curr_tick;
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
        }

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC2_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2) - 1, 200);
        }
}

void pot_read() {
    // Potentiometer mode LED indicator
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, pot_mode);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !pot_mode);

    if (abs(adc_buf[0] - pot_value) >= POT_DEAD_ZONE) {
        pot_value = adc_buf[0];
    }

    if (pot_mode) {
        telemetry.engine_temp = pot_value * MAX_TEMP / POT_RESOLUTION;
    }
    else {
        info.fuel_level = pot_value;
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

void uart_print() {
    // char msg[128];
    
    // snprintf(msg, sizeof(msg),
    // "G: %u, L: %i, R: %i, LM: %i, PM: %u, FL: %hu, ET: %u, E1: %u, E2: %u.\r\n",
    // gear, turn_left, turn_right, light_mode, pot_mode, fuel_level,
    // engine_temp, rpm, speed);

    // HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void vechicle_info() {

    //Fuel range and fuel level notification handler
    float fuel_level_liters = fuel_level_to_liters(info.fuel_level);
    info.fuel_range = fuel_level_liters / (float)FUEL_CONSUPTION * 100;

    if (fuel_level_liters <= CRITICAL_FUEL_LEVEL) {
        send_CAN_notification(&low_fuel_alert, NOTI_SET);
        info.low_fuel_lamp = true;
    }
    // Clear notification only once to prevent repeating CAN sending
    else if (fuel_level_liters > CRITICAL_FUEL_LEVEL && low_fuel_alert.status == 1)
    {
        send_CAN_notification(&low_fuel_alert, NOTI_CLEAR);
        info.low_fuel_lamp = false;
    }

    // Engine temp notification handler
    telemetry.coolant_temp = (uint8_t)(telemetry.engine_temp * 0.8);

    if(telemetry.engine_temp >= CRITICAL_TEMP) {
        send_CAN_notification(&engine_temp_alert, NOTI_SET);
    } 
    // Clear notification only once to prevent repeating CAN sending
    else if (telemetry.engine_temp < CRITICAL_TEMP && engine_temp_alert.status == 1) {
        send_CAN_notification(&engine_temp_alert, NOTI_CLEAR);
    }
    
}

float fuel_level_to_liters(uint16_t fuel_level) {
    return (float)info.fuel_level * TANK_VOLUME / 4096.0;
}