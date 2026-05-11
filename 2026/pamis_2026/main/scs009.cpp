#include "scs009.h"
#include "SAP_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace scs009
{
    enum eRegister : uint8_t
    {
        R_GoalPosition = 0x2A,
        R_CurrentPosition = 0x38,
    };

    void move_scs(uint8_t id, uint16_t pos)
    {
        sap_pkt_t pkt = {
            .id = id,
            .len = 5,
            .instruction = SAP_WRITE};

        pkt.params[0] = R_GoalPosition;
        pkt.params[1] = (uint8_t)(pos >> 8);
        pkt.params[2] = (uint8_t)(pos & 0xFF);

        sap_send_pkt(&pkt);
    }

    int16_t readPos_scs(uint8_t id)
    {
        sap_pkt_t pkt = {
            .id = id,
            .len = 4,
            .instruction = SAP_READ};
        pkt.params[0] = R_CurrentPosition;
        pkt.params[1] = 2;

        sap_pkt_t rcv_pkt;
        while (readPacket(&rcv_pkt, 0) == pdTRUE)
            ;

        sap_send_pkt(&pkt);

        if (readPacket(&rcv_pkt, 20 / portTICK_PERIOD_MS) == pdTRUE)
        {
            uint16_t pos = (uint16_t)(rcv_pkt.params[0] << 8) | rcv_pkt.params[1];
            return (int16_t)pos;
        }

        return -1;
    }
}