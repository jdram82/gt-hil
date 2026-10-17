/*
 * hil_port_stm32.c - HilPort for the Nucleo-F446RE (STM32 HAL).
 *
 * Needs a HAL project with
 *   USART1  (PA9 TX, PA10 RX)  115200, Word Length "9 bits (including parity)",
 *           parity EVEN, 1 stop bit  -> this is 8E1 on the wire; handle: huart1
 * and the DWT cycle counter (enabled in hil_port_stm32).
 */
#include "main.h"
#include "../common/hil_master.h"

extern UART_HandleTypeDef huart1;

static int p_write(void *ctx, const uint8_t *buf, int len)
{
    (void)ctx;
    return (HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 20) == HAL_OK) ? len : 0;
}

static int p_read_frame(void *ctx, uint8_t *buf, int max, int expect, uint32_t timeout_ms)
{
    (void)ctx; (void)expect;
    uint16_t got = 0;
    /* returns when the line goes idle after the first byte, or on timeout */
    HAL_UARTEx_ReceiveToIdle(&huart1, buf, (uint16_t)max, &got, timeout_ms);
    return (int)got;
}

static void p_flush(void *ctx)
{
    (void)ctx;
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
}

static uint32_t p_now_us(void *ctx)
{
    (void)ctx;
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}

HilPort hil_port_stm32(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     /* cycle counter for timing */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HilPort p = { 0, p_write, p_read_frame, p_flush, p_now_us };
    return p;
}
