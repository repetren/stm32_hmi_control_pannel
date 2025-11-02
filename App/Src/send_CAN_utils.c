#include "can.h"
#include "tim.h"

#include "send_CAN_utils.h"
#include "constants.h"


CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

void can_init() {
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
}

void send_CAN_telemetry(volatile TelemetryData *telemetry) {
    TxHeader.DLC = 7;
    TxHeader.StdId = 0x100;

    TxData[0] = telemetry->gear;
    TxData[1] = telemetry->speed >> 8 & 0xFF; // MSB
    TxData[2] = telemetry->speed & 0xFF; // LSB
    TxData[3] = telemetry->rpm >> 8 & 0xFF;
    TxData[4] = telemetry->rpm & 0xFF;
    TxData[5] = telemetry->engine_temp;
    TxData[6] = telemetry->coolant_temp;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void send_CAN_light(volatile LightsData *lights) {
    TxHeader.DLC = 1;
    TxHeader.StdId = 0x110;

    TxData[0] = (lights->turn_left & 0x01) << 7;
    TxData[0] |= (lights->turn_right & 0x01) << 6;
    TxData[0] |= (lights->light_mode & 0x03) << 4;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void send_CAN_fuel(VehicleInfo *info) {
    TxHeader.DLC = 6;
    TxHeader.StdId = 0x120;

    TxData[0] = info->fuel_level >> 8 & 0xFF; // MSB
    TxData[1] = info->fuel_level & 0xFF; // LSB
    TxData[2] = FUEL_CONSUPTION;
    TxData[3] = info->fuel_range >> 8 & 0xFF;
    TxData[4] = info->fuel_range & 0xFF;
    TxData[5] = info->low_fuel_lamp;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void send_CAN_veh_info(VehicleInfo *info) {
    TxHeader.DLC = 6;
    TxHeader.StdId = 0x121;

    TxData[0] = info->odo >> 24 & 0xFF; // MSB
    TxData[1] = info->odo >> 16 & 0xFF; // LSB
    TxData[2] = info->odo >> 8 & 0xFF;
    TxData[3] = info->odo & 0xFF;
    TxData[4] = info->trip_a >> 8 & 0xFF;
    TxData[5] = info->trip_a & 0xFF;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void send_CAN_notification(Notification *noti, uint8_t status) {
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim13);
    uint16_t elapsed = (uint16_t)now - noti->last_tx;
    
    if (noti->status != status || elapsed >= TIMER13_PERIOD_10S) {
        noti->status = status;

        TxHeader.DLC = 3;
        TxHeader.StdId = 0x130;

        TxData[0] = noti->type;
        TxData[1] = noti->code;
        TxData[2] = noti->status;

        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

        noti->last_tx = __HAL_TIM_GET_COUNTER(&htim13);
    }
}  