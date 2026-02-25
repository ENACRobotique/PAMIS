#include "telelogs.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
//#include "esp_netif.h"
#include "wifi_setup.h"
#define TAG ""

struct sockaddr_in dest_addr;
int addr_family = 0;
int ip_protocol = 0;
struct timeval timeout;
int sock = 0;

static bool is_initialized = false;

void telelogs_init() {
    char* teleplot_ip = read_string_from_nvs("teleplot_ip");
    uint16_t teleplot_port;
    esp_err_t ret = read_u16_from_nvs("teleplot_port", &teleplot_port);
    if(teleplot_ip != NULL || ret == ESP_OK) {
        printf("Reading teleplot ip from NVS: %s:%u\n", teleplot_ip, teleplot_port);
        dest_addr.sin_addr.s_addr = inet_addr(teleplot_ip);
        dest_addr.sin_port = htons(teleplot_port);
    }
    else {
        dest_addr.sin_addr.s_addr = inet_addr("192.168.42.201");
        dest_addr.sin_port = htons(47269);
    }
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

void telelogs_send_string(const char* name, const char* str) {
    if(!is_initialized) {
        return;
    }
    char data[500];
    int len = snprintf(data, 500, "%s:%s|t", name, str);
    int err = sendto(sock, data, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return;
    }
    //ESP_LOGI(TAG, "Message sent");
}


void telelogs_send_float(const char* name, float value) {
    if(!is_initialized) {
        return;
    }
    char buffer[50];
    int len = snprintf(buffer, 50, "%s:%f\n", name, value);
    int err = sendto(sock, buffer, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    //ESP_LOG_BUFFER_CHAR("", buffer, len);
}