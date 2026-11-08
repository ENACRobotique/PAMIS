#include "SAP_controller.h"
#include "driver/uart.h"
#include "config.h"
#include "string.h"
#include "esp_log.h"
#include "utils.h"

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;
sap_pkt_t pending_rcv_pkt;
sap_pkt_t rcv_pkt;
SemaphoreHandle_t bsem;

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

static void sap_readerTask(void *arg);

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

    xTaskCreate( sap_readerTask, "sap_reader", configMINIMAL_STACK_SIZE + 1024, NULL, 2, NULL);
    return ret;
}

static void sap_readerTask(void *arg)
{
    while (true)
    {
        uint16_t rcv_header = 0;
        while (rcv_header != 0xFFFF)
        {
            rcv_header = rcv_header << 8;
            int nb_data = uart_read_bytes(SAP_UART, &rcv_header, 1, pdMS_TO_TICKS(10));
            if (nb_data == 0)
            {
                rcv_header = 0;
            }
        }
        
        int nb_data = uart_read_bytes(SAP_UART, &(pending_rcv_pkt.id), 2, pdMS_TO_TICKS(10));
        if (nb_data != 2 || pending_rcv_pkt.len < 2 || pending_rcv_pkt.len > SAP_MAX_DATA_LEN + 2)
        {
            continue;
        }
        
        nb_data = uart_read_bytes(SAP_UART, &(pending_rcv_pkt.instruction), pending_rcv_pkt.len, pdMS_TO_TICKS(10));
        if (nb_data != pending_rcv_pkt.len)
        {
            continue;
        }

        uint8_t checksum = (pending_rcv_pkt.id + pending_rcv_pkt.len + pending_rcv_pkt.instruction);
        for (int i = 0; i < pending_rcv_pkt.len - 2; i++)
        {
            checksum += pending_rcv_pkt.params[i];
        }
        checksum = ~checksum;
        if (pending_rcv_pkt.params[pending_rcv_pkt.len - 2] != checksum)
        {
            // printf("bad checksum\n");
            continue;
        }

        // take the semaphore to invalidate previously received packet
        xSemaphoreTake(bsem, 0);
        // copy received packet
        memcpy(&rcv_pkt, &pending_rcv_pkt, sizeof(sap_pkt_t));
        // signal new incoming packet
        xSemaphoreGive(bsem);

        //printf("pkt received (type %u)\n", pending_rcv_pkt.instruction);
    }
}

int readPacket(sap_pkt_t *pkt, TickType_t timeout)
{
    if (xSemaphoreTake(bsem, timeout) == pdTRUE)
    {
        if(pkt) {
            memcpy(pkt, &rcv_pkt, sizeof(rcv_pkt));
        }
        return pdTRUE;
    };
    return pdFALSE;
}

int check_response(sap_pkt_t *req, TickType_t timeout) {
    sap_pkt_t resp;
    if(readPacket(&resp, timeout) != pdTRUE ||
        resp.id != req->id ||
        resp.len != 2
      ) {
        return -1;
    }
    return resp.instruction;
}

int sap_ping(uint8_t id)
{

    sap_pkt_t ping_pkt = {
        .id = id,
        .len = 2,
        .instruction = SAP_PING,
    };

    sap_pkt_t rcv_pkt;
    readPacket(&rcv_pkt, 0);

    if (sap_send_pkt(&ping_pkt) != ESP_OK)
    {
        ESP_LOGE("SAP", "Send packet error!");
    }
    else
    {
        ESP_LOGI("SAP", "send packet ok");
    }
    utils::sleep(1);
    if (readPacket(&rcv_pkt, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        return 1;
    }
    return 0;
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
    
    // take the semaphore to invalidate previously received packet
    xSemaphoreTake(bsem, 0);

    int ret = uart_write_bytes(SAP_UART, pkt, pkt->len + 4);

    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(10)));

    if(ret == pkt->len + 4) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}
