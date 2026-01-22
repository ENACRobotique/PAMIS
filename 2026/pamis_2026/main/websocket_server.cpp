#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include <sys/stat.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <sys/param.h>
#include "pins_config.h"
#include "locomotion.h"


extern Locomotion locomotion;

httpd_handle_t server = NULL;
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

static const char *TAG = "WebSocket Server"; // TAG for debug

#define INDEX_HTML_PATH "/spiffs/index.html"
#define LOW_RES_PNG_PATH "/spiffs/low_res.png"
#define ROBOT_PNG_PATH "/spiffs/robot.png"

char index_html[8192];
ssize_t low_res_size;
char low_res_png[200000];
ssize_t robot_size;
char robot_png[8192];

//char response_data[8192];

static void load_web_page_buffer(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    memset((void *)index_html, 0, sizeof(index_html));
    struct stat st;
    if (stat(INDEX_HTML_PATH, &st))
    {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen(INDEX_HTML_PATH, "r");
    if (fread(index_html, st.st_size, 1, fp) == 0)
    {
        ESP_LOGE(TAG, "fread failed");
    }
    fclose(fp);

    stat(LOW_RES_PNG_PATH, &st);
    low_res_size = st.st_size;
    fp = fopen(LOW_RES_PNG_PATH, "r");
    fread(low_res_png, st.st_size, 1, fp);
    fclose(fp);

    stat(ROBOT_PNG_PATH, &st);
    robot_size = st.st_size;
    fp = fopen(ROBOT_PNG_PATH, "r");
    fread(robot_png, st.st_size, 1, fp);
    fclose(fp);
}

/**
 * GET on /
 */
esp_err_t get_req_handler(httpd_req_t *req)
{
    int response = httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return response;
}

/**
 * GET on /
 */
esp_err_t get_req_low_res_handler(httpd_req_t *req)
{
    int response = httpd_resp_send(req, low_res_png, low_res_size);
    return response;
}

/**
 * GET on /
 */
esp_err_t get_req_robot_handler(httpd_req_t *req)
{
    int response = httpd_resp_send(req, robot_png, robot_size);
    return response;
}

/**
 * GET on /led
 */
esp_err_t get_req_led_handler(httpd_req_t *req) {
    char* data;
    if(gpio_get_level(LED2)) {
        data = "ON";
    } else {
        data = "OFF";
    }
    ESP_LOGI(TAG, "LED request, responding %s", data);
    int response = httpd_resp_send(req, data, HTTPD_RESP_USE_STRLEN);
    return response;
}


static void ws_async_send_led_state(void *arg)
{
    httpd_ws_frame_t ws_pkt = {0};

    int led_state = gpio_get_level(LED2);
    
    char buff[4];
    sprintf(buff, "%d",led_state);
    
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK) {
        return;
    }

    for (int i = 0; i < fds; i++) {
        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
        }
    }
}



void ws_async_send_robot_pos()
{
    httpd_ws_frame_t ws_pkt = {0};

    int led_state = gpio_get_level(LED2);
    gpio_set_level(LED2, !led_state);
    
    char buff[100];
    Position pos = locomotion.getPos();
    sprintf(buff, "{\"led\":%d, \"x\":%02f, \"y\":%02f, \"theta\":%02f}",!led_state, pos.x, pos.y, pos.theta);
    
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
    size_t fds = max_clients;
    int client_fds[max_clients];

    esp_err_t ret = httpd_get_client_list(server, &fds, client_fds);

    if (ret != ESP_OK) {
        return;
    }

    for (int i = 0; i < fds; i++) {
        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
        }
    }
}




static esp_err_t handle_ws_req(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {0};

    // Calling httpd_ws_recv_frame() with max_len as 0 will give actual frame size in pkt->len.
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (ws_pkt.len)
    {
        ws_pkt.payload = (uint8_t*)malloc(ws_pkt.len+1);
        
        if (ws_pkt.payload == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for payload_buf");
            return ESP_ERR_NO_MEM;
        }
        
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(ws_pkt.payload);
            return ret;
        }
        
        ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
            ws_pkt.payload[ws_pkt.len] = '\0';  // NULL terminate the string
            ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);

            if(strncmp((char *)ws_pkt.payload, "toggle", ws_pkt.len) == 0) {
                gpio_set_level(LED2, !gpio_get_level(LED2));
                httpd_queue_work(req->handle, ws_async_send_led_state, NULL);
            }
            else if (strncmp((char *)ws_pkt.payload, "forward", ws_pkt.len) == 0) {
                locomotion.move(300, 0);
            }
            else if (strncmp((char *)ws_pkt.payload, "backward", ws_pkt.len) == 0) {
                locomotion.move(-300, 0);
            }
            else if (strncmp((char *)ws_pkt.payload, "left", ws_pkt.len) == 0) {
                locomotion.move(0, M_PI_2);
            }
            else if (strncmp((char *)ws_pkt.payload, "right", ws_pkt.len) == 0) {
                locomotion.move(0, -M_PI_2);
            }
            else if (strncmp((char *)ws_pkt.payload, "dis_step", ws_pkt.len) == 0) {
                locomotion.enableSteppers(false);
            }
            else if (strncmp((char *)ws_pkt.payload, "en_step", ws_pkt.len) == 0) {
                locomotion.enableSteppers(true);
            }
        }
    }

    free(ws_pkt.payload);
    return ESP_OK;
}

httpd_handle_t setup_websocket_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t uri_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_req_handler,
        .user_ctx = NULL};
    
    httpd_uri_t uri_get_low_res = {
        .uri = "/low_res.png",
        .method = HTTP_GET,
        .handler = get_req_low_res_handler,
        .user_ctx = NULL};
    
    httpd_uri_t uri_get_robot = {
        .uri = "/robot.png",
        .method = HTTP_GET,
        .handler = get_req_robot_handler,
        .user_ctx = NULL};
    
    httpd_uri_t uri_led_get = {
        .uri = "/led",
        .method = HTTP_GET,
        .handler = get_req_led_handler,
        .user_ctx = NULL};

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handle_ws_req,
        .user_ctx = NULL,
        .is_websocket = true};
    
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_get_low_res);
        httpd_register_uri_handler(server, &uri_get_robot);
        httpd_register_uri_handler(server, &uri_led_get);
        httpd_register_uri_handler(server, &ws);
    }

    return server;
}

void start_web_server()
{
    load_web_page_buffer();
    server = setup_websocket_server();
    ESP_LOGI(TAG, "ESP32 ESP-IDF WebSocket Web Server is running ... ...\n");
}