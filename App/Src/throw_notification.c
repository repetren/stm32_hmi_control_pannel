#include <stdint.h>

#include "send_CAN_utils.h"
#include "constants.h"
#include "fuel_and_temp_utils.h"

Notification low_fuel_alert = {
    VEH_INFO,
    LOW_FUEL_NOTIF_CODE,
    NOTI_CLEAR,
    0};
Notification engine_temp_alert = {
    VEH_INFO,
    HIGH_ENGINE_TEMP_NOTIF_CODE,
    NOTI_CLEAR,
    0};

void check_temp_for_notification(volatile TelemetryData *telemetry) {
    // Fake value for coolant temp
    telemetry->coolant_temp = (uint8_t)(telemetry->engine_temp * 0.8); 

    if(telemetry->engine_temp >= CRITICAL_TEMP) {
        send_CAN_notification(&engine_temp_alert, NOTI_SET);
    } 
    // Clear notification only once to prevent repeating CAN sending
    else if (telemetry->engine_temp < CRITICAL_TEMP && engine_temp_alert.status == 1) {
        send_CAN_notification(&engine_temp_alert, NOTI_CLEAR);
    }
}

void check_fuel_level_for_notification(volatile VehicleInfo *info) {
    if (info->fuel_liters <= CRITICAL_FUEL_LEVEL) {
        send_CAN_notification(&low_fuel_alert, NOTI_SET);
        info->low_fuel_lamp = true;
    }
    // Clear notification only once to prevent repeating CAN sending
    else if (info->fuel_level > CRITICAL_FUEL_LEVEL && low_fuel_alert.status == 1)
    {
        send_CAN_notification(&low_fuel_alert, NOTI_CLEAR);
        info->low_fuel_lamp = false;
    }
}