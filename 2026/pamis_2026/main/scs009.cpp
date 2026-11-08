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

    int move(uint8_t id, uint16_t pos)
    {
        sap_pkt_t pkt = {
            .id = id,
            .len = 5,
            .instruction = SAP_WRITE};

        pkt.params[0] = R_GoalPosition;
        pkt.params[1] = static_cast<uint8_t>(pos >> 8);
        pkt.params[2] = static_cast<uint8_t>(pos);

        if (sap_send_pkt(&pkt) != ESP_OK)
        {
            return -1;
        }

        return check_response(&pkt, pdMS_TO_TICKS(20));
    }

    int16_t readPos(uint8_t id)
    {
        sap_pkt_t pkt = {
            .id = id,
            .len = 4,
            .instruction = SAP_READ};
        pkt.params[0] = R_CurrentPosition;
        pkt.params[1] = 2;

        sap_pkt_t rcv_pkt;

        if (sap_send_pkt(&pkt) != ESP_OK)
        {
            return -1;
        }

        if (readPacket(&rcv_pkt, pdMS_TO_TICKS(30)) == pdTRUE &&
            rcv_pkt.id == id &&
            rcv_pkt.len == 4 &&
            rcv_pkt.instruction == 0)
        {
            uint16_t pos = (static_cast<uint16_t>(rcv_pkt.params[0]) << 8) |
                           static_cast<uint16_t>(rcv_pkt.params[1]);
            return static_cast<int16_t>(pos);
        }

        return -1;
    }
}
