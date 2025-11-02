#include "gpio.h"
#include "send_CAN_utils.h"
#include <stdint.h>
#include <stdlib.h>
#include "fuel_and_temp_utils.h"
#include "constants.h"

uint16_t pot_value = 0;
volatile uint8_t pot_mode = 0;
volatile uint16_t adc_buf[ADC_BUF_LEN] = {0};

void pot_read(volatile TelemetryData *telemetry, VehicleInfo *info, volatile uint16_t pot_mode) {
    // Potentiometer mode LED indicator
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, pot_mode);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, !pot_mode);

    if (abs(adc_buf[0] - pot_value) >= POT_DEAD_ZONE) {
        pot_value = adc_buf[0];
    }

    if (pot_mode) {
        telemetry->engine_temp = pot_value * MAX_TEMP / POT_RESOLUTION;
    }
    else {
        info->fuel_level = pot_value;
    }
}

void fuel_level_to_liters(VehicleInfo *info) {
    info->fuel_liters = info->fuel_level * TANK_VOLUME / 4096.0;
}

void fuel_liter_to_range(VehicleInfo *info) {
    info->fuel_range = info->fuel_liters / (float)FUEL_CONSUPTION * 100;
}