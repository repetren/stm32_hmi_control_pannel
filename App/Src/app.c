#include "app.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

uint32_t press_count = 0;
volatile bool up_pressed = false;
uint8_t gear = 0;
uint8_t old_gear = 0;

char msg[32];

void app()
{

    while(1) {
        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        static uint32_t i = 0;

        if (gear != old_gear) {
            sprintf(msg, "Gear: %i.\r\n", gear);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

            old_gear = gear;
        }

        static uint32_t last_mc = 0;

        if (up_pressed) {
            uint32_t now = HAL_GetTick();
            if ((now - last_mc) > 150) {
                gear++;
                last_mc = now;
            }
            up_pressed = false;
        }

        HAL_Delay(50);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BTN_UP_Pin)
        up_pressed = true;
}