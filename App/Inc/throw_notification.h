#ifndef __THROW_NOTIFICATION_H__
#define __THROW_NOTIFICATION_H__

void check_temp_for_notification(volatile TelemetryData *telemetry);
void check_fuel_level_for_notification(volatile VehicleInfo *info);

#endif // __THROW_NOTIFICATION_H__