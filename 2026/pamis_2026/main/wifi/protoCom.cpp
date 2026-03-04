#include "telelogs.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "esp_netif.h"
#include "wifi_setup.h"
#define TAG ""
#include "locomotion.h"
#include "proto/pamis.h"
#include "BytesWriteBuffer.h"

static struct sockaddr_in dest_addr;
static int addr_family = 0;
static int ip_protocol = 0;
static struct timeval timeout;
static int sock = 0;

static bool is_initialized = false;


BytesWriteBuffer msgBuffer;


void protocom_init() {
    dest_addr.sin_addr.s_addr = inet_addr("192.168.42.201");
    dest_addr.sin_port = htons(12123);
    dest_addr.sin_family = AF_INET;
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    // Set timeout
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
    is_initialized = true;
}

void protocom_send_pos(Position pos) {
    if(!is_initialized) {
        return;
    }
    
    enac::PamiMessage pamiMsg;
    pamiMsg.set_msg_type(enac::PamiMessage::MsgType::STATUS);
    auto& ppos = pamiMsg.mutable_pos();
    ppos.set_x(pos.x);
    ppos.set_y(pos.y);
    ppos.set_theta(pos.theta);

    msgBuffer.clear();
    pamiMsg.serialize(msgBuffer);

    size_t buf_size = msgBuffer.get_size();
    uint8_t* data = msgBuffer.get_data();


    int err = sendto(sock, data, buf_size, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return;
    }
}
