#pragma once
#include "inttypes.h"
#include "stddef.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

constexpr size_t SAP_MAX_DATA_LEN = 10;
constexpr uint8_t SAP_BROADCAST_ID = 254;


typedef enum {
    SAP_PING = 0x01,
    SAP_READ = 0x02,
    SAP_WRITE = 0x03,
    // SAP_REG_WRITE = 0x04,
    // SAP_ACTION = 0x05,
    // SAP_RESET = 0x06,
    // SAP_REBOOT = 0x08,
    // SAP_SYNC_WRITE = 0x83,
    // SAP_BLUK_READ = 0x92
} sap_instruction_t;

typedef struct __attribute__((packed)) {
    uint16_t STX;
    uint8_t id;
    uint8_t len;
    uint8_t instruction;        // or error for status packets
    uint8_t params[SAP_MAX_DATA_LEN+1]; // params + checksum
} sap_pkt_t;



esp_err_t sap_init(int baudrate);
int sap_ping(uint8_t id);

esp_err_t sap_send_pkt(sap_pkt_t* pkt);

int readPacket(sap_pkt_t* pkt, TickType_t timeout);
