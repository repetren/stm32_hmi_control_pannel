#include <stdint.h>
#include <stdlib.h>

#include "usart.h"
#include "i2c.h"
#include "send_CAN_utils.h"
#include "stm32f4xx_hal_i2c.h"

#include "encoder_utils.h"
#include "constants.h"


int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range,
                      int16_t min_value, int16_t max_value) {
    uint16_t step = (uint32_t)encoder_angle * range / 16384;

    int16_t diff = enc->old_step - step;

    if (step != enc->old_step && abs(diff) < range / 2) {
        enc->output += diff;
    }

    enc->old_step = step;

    if (enc->output > max_value) enc->output = max_value;
    // enc->output = fmin(enc->output, max_value);
    else if (enc->output < min_value) enc->output = min_value;
    // enc->output = fmax(enc->output, min_value);

    return enc->output;
}

void encoders_read(EncoderData *encoder_1, EncoderData *encoder_2, volatile TelemetryData *telemetry) {
    HAL_I2C_Mem_Read(&hi2c3, (ENC1_ADDR << 1), ENC_REG_HIGH, I2C_MEMADD_SIZE_8BIT,
                                &encoder_1->high_byte, 1, 5);
    HAL_I2C_Mem_Read(&hi2c3, (ENC1_ADDR << 1), ENC_REG_LOW, I2C_MEMADD_SIZE_8BIT,
                                &encoder_1->low_byte, 1, 5);
    HAL_I2C_Mem_Read(&hi2c3, (ENC2_ADDR << 1), ENC_REG_HIGH, I2C_MEMADD_SIZE_8BIT,
                                &encoder_2->high_byte, 1, 5);
    HAL_I2C_Mem_Read(&hi2c3, (ENC2_ADDR << 1), ENC_REG_LOW, I2C_MEMADD_SIZE_8BIT,
                                &encoder_2->low_byte, 1, 5);

    uint16_t angle_encoder_1 = ((uint16_t)encoder_1->high_byte << 6
                                | (encoder_1->low_byte & 0x3F));
    uint16_t angle_encoder_2 = ((uint16_t)encoder_2->high_byte << 6
                                | (encoder_2->low_byte & 0x3F));

    telemetry->rpm = encoder_remap(encoder_1, angle_encoder_1, 2048, 0, 4096);
    telemetry->speed = encoder_remap(encoder_2, angle_encoder_2, 100, 0, MAX_SPEED);
}

void encoder_check() {
        static const char msg1[] = "Encoder 1 not ready\r\n";
        static const char msg2[] = "Encoder 2 not ready\r\n";

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC1_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg1, sizeof(msg1) - 1, 200);
        }

        if (HAL_I2C_IsDeviceReady(&hi2c3, (ENC2_ADDR << 1), 1, 100) != HAL_OK) {
            HAL_UART_Transmit(&huart2, (uint8_t*)msg2, sizeof(msg2) - 1, 200);
        }
}