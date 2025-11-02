#ifndef __SEND_CAN_UTILS_H__
#define __SEND_CAN_UTILS_H__

#include <stdint.h>

#include "constants.h"
#include <stdbool.h>

// Vehicle info
typedef struct {
    uint32_t odo;         //km
    uint16_t trip_a;      // km
    uint16_t fuel_level;  // from potentiometer
    uint16_t fuel_range;  // km
    uint16_t fuel_liters; // l
    bool low_fuel_lamp;
} VehicleInfo;

// Notification
enum Notification_type {
    TELEMETRY = 1,
    VEH_INFO = 2,
    CONNECTION = 3,
};

enum Notification_status {
    NOTI_CLEAR = 0,
    NOTI_SET = 1,
};

typedef struct {
    uint8_t type;
    uint8_t code;
    uint8_t status;
    uint16_t last_tx;
} Notification;

// Telemetry
typedef struct {
    uint8_t  gear;         // 0-8
    uint8_t  engine_temp;  // deg Celsius
    uint8_t  coolant_temp; // deg Celsius
    uint16_t rpm;          // raw units 0-4096 from encoder
    uint16_t speed;        // km/h
} TelemetryData;

// Lights objects
typedef struct {
    uint8_t light_mode;
    bool turn_left;
    bool turn_right;
} LightsData;

void can_init();
void send_CAN_telemetry(volatile TelemetryData *telemetry);
void send_CAN_light(volatile LightsData *lights);
void send_CAN_fuel(VehicleInfo *info);
void send_CAN_veh_info(VehicleInfo *info);
void send_CAN_notification(Notification *noti, uint8_t status);

#endif // __SEND_CAN_UTILS_H__