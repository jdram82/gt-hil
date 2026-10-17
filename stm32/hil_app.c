/*
 * hil_app.c - glue between the STM32 HAL project and hil_master.
 *
 * In main.c (CubeIDE generated):
 *     #include "hil_app.h"
 *     ... after MX_USART2_UART_Init():   hil_app_init();
 *     ... inside while (1):              hil_app_loop();
 *
 * Every 50 ms it runs one controller cycle. Every 2 s it prints a status line
 * on USART2 (the ST-LINK virtual COM port, 115200 8N1) and toggles the LED.
 */
#include "main.h"
#include "hil_app.h"
#include "../common/hil_master.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
HilPort hil_port_stm32(void);

static HilMaster g_m;
static uint32_t  g_next, g_next_print;

void hil_app_init(void)
{
    hil_master_init(&g_m, hil_port_stm32(), 1);
    g_next = HAL_GetTick() + HIL_CONTROL_PERIOD_MS;
    g_next_print = g_next + 2000u;
    const char *s = "\r\ngt-hil controller up (register map v1.0)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

void hil_app_loop(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_next) < 0) return;
    g_next += HIL_CONTROL_PERIOD_MS;                       /* fixed 50 ms grid, no drift */
    hil_master_cycle(&g_m);

    if ((int32_t)(now - g_next_print) >= 0) {
        g_next_print += 2000u;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        char line[160];
        int n = snprintf(line, sizeof line,
            "cyc %lu fail %lu to %lu crc %lu | st %d fo %d | n %.0f rpm | fsr %.3f | cycle %lu us (max %lu) ctrl %lu us\r\n",
            (unsigned long)g_m.cycles, (unsigned long)g_m.failed_cycles, (unsigned long)g_m.timeouts,
            (unsigned long)g_m.crc_errors, (int)g_m.ctrl.state, g_m.ctrl.first_out,
            g_m.ctrl.speed_voted_rpm, g_m.ctrl.fsr_out,
            (unsigned long)g_m.last_cycle_us, (unsigned long)g_m.max_cycle_us, (unsigned long)g_m.last_ctrl_us);
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 100);
    }
}
