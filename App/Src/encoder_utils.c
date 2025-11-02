#include <stdint.h>
#include <stdlib.h>

#include "encoder_utils.h"

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