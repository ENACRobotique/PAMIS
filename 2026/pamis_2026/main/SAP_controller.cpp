#include "SAP_controller.h"
#include "driver/uart.h"
#include "config.h"

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

esp_err_t sap_init(int baudrate)
{
    uart_driver_install(SAP_UART, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
    uart_config.baud_rate = baudrate;
    esp_err_t ret = uart_param_config(SAP_UART, &uart_config);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = uart_set_mode(SAP_UART, UART_MODE_RS485_HALF_DUPLEX);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = uart_set_pin(SAP_UART, TX1, RX1, RTS, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return ret;
}

esp_err_t sap_send_pkt(sap_pkt_t *pkt)
{
    pkt->STX = 0xFFFF;
    uint8_t checksum = (pkt->id + pkt->len + pkt->instruction);
    for (int i = 0; i < pkt->len - 2; i++)
    {
        checksum += pkt->params[i];
    }
    pkt->params[pkt->len - 2] = ~checksum;
    esp_err_t ret = uart_write_bytes(SAP_UART, pkt, pkt->len + 4);
    return ret;
}