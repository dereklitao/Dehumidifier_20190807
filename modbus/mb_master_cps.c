#include "mb_config.h"

modbus_master master_cps;
static osSemaphoreId uart_idle_sem;
static osMutexId uart_source_mut;

void master_cps_set_tx(void)
{
    HAL_GPIO_WritePin(UEn3_GPIO_Port, UEn3_Pin, GPIO_PIN_SET);
}

void master_cps_set_rx(void)
{
    HAL_GPIO_WritePin(UEn3_GPIO_Port, UEn3_Pin, GPIO_PIN_RESET);
}

void master_cps_uart_idle(void)
{
    if (((READ_REG(master_cps.uart->Instance->SR) & USART_SR_IDLE) != RESET) && ((READ_REG(master_cps.uart->Instance->CR1) & USART_CR1_IDLEIE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(master_cps.uart);
        master_cps.rx_len = MODBUS_BUFFER_LENGTH - master_cps.uart->hdmarx->Instance->CNDTR;
        osSemaphoreRelease(uart_idle_sem);
    }
}

uint8_t master_cps_send_receive(uint16_t timeout)
{
    master_cps.buff_len = master_cps.tx_len;
    master_crc16(&master_cps, master_cps.tx_buf);
    master_cps.tx_buf[master_cps.tx_len++] = master_cps.crc_hi;
    master_cps.tx_buf[master_cps.tx_len++] = master_cps.crc_lo;
    osSemaphoreWait(uart_idle_sem, 0);
    HAL_UART_Receive_DMA(master_cps.uart, master_cps.rx_buf, MODBUS_BUFFER_LENGTH);
    master_cps.master_set_tx();
    HAL_UART_Transmit_DMA(master_cps.uart, master_cps.tx_buf, master_cps.tx_len);
    if (osSemaphoreWait(uart_idle_sem, timeout) == osOK)
    {
        master_cps.status = 1;
    }
    else
    {
        master_cps.status = 0;
    }
    HAL_UART_DMAStop(master_cps.uart);
    return master_cps.status;
}

void csro_master_cps_init(UART_HandleTypeDef *uart)
{
    osSemaphoreDef(uart_idle_semaphore);
    uart_idle_sem = osSemaphoreCreate(osSemaphore(uart_idle_semaphore), 1);
    osMutexDef(uart_source_mutex);
    uart_source_mut = osMutexCreate(osMutex(uart_source_mutex));
    master_cps.uart = uart;
    master_cps.master_set_tx = master_cps_set_tx;
    master_cps.master_set_rx = master_cps_set_rx;
    master_cps.master_uart_idle = master_cps_uart_idle;
    master_cps.master_send_receive = master_cps_send_receive;

    __HAL_UART_ENABLE_IT(master_cps.uart, UART_IT_IDLE);
}

uint16_t cps_result[10];
void csro_master_cps_read_task(void)
{
    if (osMutexWait(uart_source_mut, osWaitForever) == osOK)
    {
        master_cps.slave_id = 0x10;
        master_cps.read_addr = 0x10;
        master_cps.read_qty = 10;
        master_read_holding_regs(&master_cps, cps_result);
        osMutexRelease(uart_source_mut);
    }
}
