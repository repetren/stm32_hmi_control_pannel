#include "unity.h"
#include "remap.h"
#include "unity_internals.h"
#include <stdint.h>

EncoderData encoder;

uint16_t range = 2048;
uint16_t min_value = 0;
uint16_t max_value = 4096;

void setUp(void) {
    encoder.output = 0;
}
void tearDown(void) {}

void test_encoder_unchanged() {
    uint16_t angle = 0;
    encoder.old_step = 0;
    uint16_t expected_value = 0;

    uint16_t result = encoder_remap(&encoder, angle, range, min_value, max_value);
    TEST_ASSERT_EQUAL_INT(expected_value, result);
}

void test_encoder_over_half_period() {
    uint16_t angle = 9000;
    encoder.old_step = 0;
    uint16_t expected_value = 0;

    uint16_t result = encoder_remap(&encoder, angle, range, min_value, max_value);
    TEST_ASSERT_EQUAL_INT(expected_value, result);
}

void test_encoder_up_to_half_period() {
    uint16_t angle = 10384;
    encoder.old_step = range;
    uint16_t expected_value = 750;

    uint16_t result = encoder_remap(&encoder, angle, range, min_value, max_value);
    TEST_ASSERT_EQUAL_INT(expected_value, result);
}

void test_encoder_mid_to_zero() {
    uint16_t angle = 8300;
    encoder.old_step = range;
    uint16_t expected_value = 0;

    uint16_t result = encoder_remap(&encoder, angle, range, min_value, max_value);
    TEST_ASSERT_EQUAL_INT(expected_value, result);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_encoder_unchanged);
    RUN_TEST(test_encoder_over_half_period);
    RUN_TEST(test_encoder_up_to_half_period);
    RUN_TEST(test_encoder_mid_to_zero);
    return UNITY_END();
}