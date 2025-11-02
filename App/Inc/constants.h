#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define DEBOUNCE_TRESHHOLD 200 // in milliseconds

#define TANK_VOLUME 20
#define FUEL_CONSUPTION 5
#define CRITICAL_FUEL_LEVEL 2

#define POT_RESOLUTION 4095
#define ADC_BUF_LEN 1
#define POT_DEAD_ZONE 25 // smoothing input values

// I2C Encoders
#define ENC1_ADDR     0x40
#define ENC2_ADDR     0x41
#define ENC_REG_HIGH  0xFE
#define ENC_REG_LOW   0xFF

#define TIMER13_PER_SEC 5000
#define TIMER13_PERIOD_10S (10 * TIMER13_PER_SEC)

#define TIMER14_PER_SEC 10000

#define MAX_TEMP 140
#define MAX_SPEED 280
#define MAX_GEAR 8
#define MIN_GEAR 0
#define CRITICAL_TEMP 120

#define DEMO_ODO 34500
#define TRIP_A_DEMO 40

enum {
    LOW_FUEL_NOTIF_CODE = 0,
    HIGH_ENGINE_TEMP_NOTIF_CODE = 1
};

#endif /* __CONSTANTS_H__ */