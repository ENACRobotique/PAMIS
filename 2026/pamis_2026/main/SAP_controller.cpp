#include "SAP_controller.h"
#include "driver/uart.h"
#include "config.h"
#include "string.h"
const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;
sap_pkt_t rcv_pkt;
SemaphoreHandle_t bsem;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

static void sap_readerTask(void* arg);

esp_err_t sap_init(int baudrate)
{
    bsem = xSemaphoreCreateBinary();
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

    xTaskCreate( sap_readerTask, "sap_reader", configMINIMAL_STACK_SIZE + 1024, NULL, 1, NULL);
    return ret;

}

static void sap_readerTask(void* arg){
    while(true){
        uint16_t rcv_header = 0;
        while(rcv_header != 0xFFFF){
            rcv_header = rcv_header<<8;
            int nb_data = uart_read_bytes(SAP_UART, &rcv_header, 1, 10 / portTICK_PERIOD_MS);
            if(nb_data == 0){
                rcv_header = 0;
            }
        }
        xSemaphoreTake(bsem, 0);
        int nb_data = uart_read_bytes(SAP_UART, &(rcv_pkt.id), 2, 10 / portTICK_PERIOD_MS);
        if(nb_data != 2){
            continue;
        }
        nb_data = uart_read_bytes(SAP_UART, &(rcv_pkt.instruction), rcv_pkt.len, 10 / portTICK_PERIOD_MS);
        if(nb_data != rcv_pkt.len){
            continue;
        }

        uint8_t checksum = (rcv_pkt.id + rcv_pkt.len + rcv_pkt.instruction);
        for (int i = 0; i < rcv_pkt.len - 2; i++)
        {
            checksum += rcv_pkt.params[i];
        }
        checksum = ~checksum;
        if(rcv_pkt.params[rcv_pkt.len - 2] != checksum){
            //printf("bad checksum\n");
            continue;
        }
        xSemaphoreGive(bsem);
        //printf("pkt received\n");
    }

}

int readPacket(sap_pkt_t* pkt, TickType_t timeout){
    if(xSemaphoreTake(bsem, timeout)==pdTRUE){
        memcpy(pkt, &rcv_pkt, sizeof(rcv_pkt));
        return pdTRUE;
    };
    return pdFALSE;
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

