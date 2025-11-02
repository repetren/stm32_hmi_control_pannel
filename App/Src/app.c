#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "gpio.h"
#include "adc.h"
#include "can.h"
#include "tim.h"

#include "constants.h"
#include "encoder_utils.h"
#include "send_CAN_utils.h"
#include "throw_notification.h"
#include "fuel_and_temp_utils.h"

// Buttons ID for debounce tick track
typedef enum {
    BTN_UP_ID,
    BTN_DOWN_ID,
    BTN_LEFT_ID,
    BTN_RIGHT_ID,
    LIGHT_MODE_ID,
    BTN_POT_MODE_ID,
    BUTTONS_NUM,
} ButtonId;

// array for buttons timestamps
static volatile uint32_t btn_prev_tick[BUTTONS_NUM] = {0};

static volatile TelemetryData telemetry = {0};
static volatile LightsData lights = {0};

static VehicleInfo info = {
    .odo = DEMO_ODO,
    .trip_a = TRIP_A_DEMO,
    .fuel_level = 0,
    .fuel_range = 0,
    .low_fuel_lamp = false,
    .fuel_liters = 0,
};

static EncoderData encoder_1 = {0};
static EncoderData encoder_2 = {0};

enum Period {
    PERIOD_100HZ = TIMER14_PER_SEC / 100,
    PERIOD_50HZ = TIMER14_PER_SEC / 50,
    PERIOD_25HZ = TIMER14_PER_SEC / 25,
};

bool check_debounce(uint16_t curr_tick, uint8_t id);

void app()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
    HAL_CAN_Start(&hcan1);

    HAL_TIM_Base_Start(&htim14);
    HAL_TIM_Base_Start(&htim13);

    uint16_t t100hz_last = __HAL_TIM_GET_COUNTER(&htim14);
    uint16_t t50hz_last = __HAL_TIM_GET_COUNTER(&htim14);
    uint16_t t25hz_last = __HAL_TIM_GET_COUNTER(&htim14);

    while(true) {
        if (__HAL_TIM_GET_COUNTER(&htim14) - t100hz_last >= PERIOD_100HZ) {
            encoders_read(&encoder_1, &encoder_2, &telemetry);
            send_CAN_telemetry(&telemetry);
            pot_read(&telemetry, &info, pot_mode);
            check_temp_for_notification(&telemetry);
            t100hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }

        if (__HAL_TIM_GET_COUNTER(&htim14) - t50hz_last >= PERIOD_50HZ) {
            send_CAN_light(&lights);
            t50hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }

        if (__HAL_TIM_GET_COUNTER(&htim14) - t25hz_last >= PERIOD_25HZ) {
            encoder_check();
            fuel_level_to_liters(&info);
            fuel_liter_to_range(&info);
            send_CAN_fuel(&info);
            send_CAN_veh_info(&info);
            check_fuel_level_for_notification(&info);
            t25hz_last = __HAL_TIM_GET_COUNTER(&htim14);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
        uint32_t curr_tick = HAL_GetTick();

        switch (GPIO_Pin)
        {
        case BTN_UP_Pin:
            if (check_debounce(curr_tick, BTN_UP_ID)
            && telemetry.gear < MAX_GEAR) {
                telemetry.gear++;
            }
            break;
        
        case BTN_DOWN_Pin:
            if (check_debounce(curr_tick, BTN_DOWN_ID)
            && telemetry.gear > MIN_GEAR) {
                telemetry.gear--;
            }
            break;

        case BTN_LEFT_Pin:
            if (check_debounce(curr_tick, BTN_LEFT_ID)) {
                lights.turn_right = false;
                lights.turn_left = !lights.turn_left;
            }
            break;

        case BTN_RIGHT_Pin:
            if (check_debounce(curr_tick, BTN_RIGHT_ID)) {
                lights.turn_left = false;
                lights.turn_right = !lights.turn_right;
            }
            break;

        case LIGHT_MODE_Pin:
            if (check_debounce(curr_tick, LIGHT_MODE_ID)) {
                if (lights.light_mode >= 0 && lights.light_mode < 3) {
                    lights.light_mode++;
                }
                else {
                    lights.light_mode = 0;
                }
            }
            break;

        case BTN_POT_MODE_Pin:
            if (check_debounce(curr_tick, BTN_POT_MODE_ID)) {
                pot_mode = !pot_mode;
            }
            break;

        default:
            break;
        }
    }

bool check_debounce(uint16_t curr_tick, uint8_t id) {
    if (curr_tick - btn_prev_tick[id] > DEBOUNCE_THRESHOLD) {
            btn_prev_tick[id] = curr_tick;
            return true;
    }
    return false;
}