#ifndef __FUEL_AND_TEMP_UTILS_H__
#define __FUEL_AND_TEMP_UTILS_H__

#include "constants.h"
extern uint16_t pot_value;
extern volatile uint8_t pot_mode;
extern volatile uint16_t adc_buf[ADC_BUF_LEN];

void pot_read(volatile TelemetryData *telemetry, VehicleInfo *info, volatile uint16_t pot_mode);
void fuel_level_to_liters(VehicleInfo *info);
void fuel_liter_to_range(VehicleInfo *info);

#endif // __FUEL_AND_TEMP_UTILS_H__