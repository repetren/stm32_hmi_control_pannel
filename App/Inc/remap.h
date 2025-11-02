#include <stdint.h>

typedef struct
{
    uint8_t high_byte;
    uint8_t low_byte;
    int16_t old_step;
    int16_t output;
} EncoderData;

int16_t encoder_remap(EncoderData *enc, uint16_t encoder_angle, uint16_t range,
                      int16_t min_value, int16_t max_value);