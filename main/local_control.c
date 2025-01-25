/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include <sys/socket.h>

#include "esp_bridge.h"
#include "esp_mesh_lite.h"

#include <cJSON.h>

#define PAYLOAD_LEN       (1456) /**< Max payload size(in bytes) */

static const char* TAG = "local_control";

#include "led_strip.h"
#define BLINK_GPIO 48

static led_strip_handle_t led_strip;

static int red = 0;
static int green = 0;
static int blue = 0;



static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);

    red = 254;
    green = 254;
    blue = 254;
}


static void light_refresh(void* arg) {
    while (1) {
        uint8_t level = esp_mesh_lite_get_level();

        for (int i = 0; i < level; i++) {
            led_strip_set_pixel(led_strip, 0, red, green, blue);
            led_strip_refresh(led_strip);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            led_strip_clear(led_strip);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        led_strip_clear(led_strip);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "esp_http_client.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

void tcp_client_write_task(void* arg)
{
    uint8_t sta_mac[6] = { 0 };
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    char mac_str[19] = { 0 };
    sprintf(mac_str, MACSTR, MAC2STR(sta_mac));

    ESP_LOGI(TAG, "HTTP client task is running");

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Declare local_response_buffer with size (MAX_HTTP_OUTPUT_BUFFER + 1) to prevent out of bound access when
        // it is used by functions like strlen(). The buffer should only be used upto size MAX_HTTP_OUTPUT_BUFFER
        char output_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = { 0 };   // Buffer to store response of http request
        int content_length = 0;

        esp_http_client_config_t config = {
            .url = "http://"CONFIG_SERVER_IP,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);

        esp_netif_ip_info_t sta_ip;
        memset(&sta_ip, 0x0, sizeof(esp_netif_ip_info_t));
        esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &sta_ip);
        uint8_t lvl = esp_mesh_lite_get_level();

        // POST Request
        cJSON* item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "mac", mac_str);
        cJSON_AddNumberToObject(item, "ip", sta_ip.ip.addr);
        cJSON_AddNumberToObject(item, "level", lvl);
        cJSON_AddNumberToObject(item, "red", 255);
        cJSON_AddNumberToObject(item, "green", 255);
        cJSON_AddNumberToObject(item, "blue", 255);

        const char* post_data = cJSON_Print(item);
        cJSON_Delete(item);

        // ESP_LOGI(TAG, "Sending data: %s", post_data);
        esp_http_client_set_url(client, "http://"CONFIG_SERVER_IP"/api/v1/registry");
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_err_t err = esp_http_client_open(client, strlen(post_data));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        }
        else {
            int wlen = esp_http_client_write(client, post_data, strlen(post_data));
            if (wlen < 0) {
                ESP_LOGE(TAG, "Write failed");
            }
            content_length = esp_http_client_fetch_headers(client);
            if (content_length < 0) {
                ESP_LOGE(TAG, "HTTP client fetch headers failed");
            }
            else {
                int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
                if (data_read >= 0) {
                    int64_t length = esp_http_client_get_content_length(client);
                    ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %"PRId64, esp_http_client_get_status_code(client), length);

                    output_buffer[length] = '\0';
                    cJSON* root = cJSON_Parse(output_buffer);
                    red = cJSON_GetObjectItem(root, "red")->valueint;
                    green = cJSON_GetObjectItem(root, "green")->valueint;
                    blue = cJSON_GetObjectItem(root, "blue")->valueint;

                    ESP_LOGI(TAG, "Light control: red = %d, green = %d, blue = %d", red, green, blue);
                    cJSON_Delete(root);
                }
                else {
                    ESP_LOGE(TAG, "Failed to read response");
                }
            }
        }
        esp_http_client_cleanup(client);
        free((void*)post_data);
    }

    ESP_LOGI(TAG, "HTTP client task is exit");
    vTaskDelete(NULL);
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary = 0;
    uint8_t sta_mac[6] = { 0 };
    wifi_ap_record_t ap_info = { 0 };
    wifi_second_chan_t second = 0;
    wifi_sta_list_t wifi_sta_list = { 0x0 };

    esp_wifi_sta_get_ap_info(&ap_info);
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG, "System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
        ", parent rssi: %d, free heap: %"PRIu32"", primary,
        esp_mesh_lite_get_level(), MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
        (ap_info.rssi != 0 ? ap_info.rssi : -120), esp_get_free_heap_size());
#if CONFIG_MESH_LITE_NODE_INFO_REPORT
    ESP_LOGI(TAG, "All node number: %"PRIu32"", esp_mesh_lite_get_mesh_node_number());
#endif /* MESH_LITE_NODE_INFO_REPORT */
    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }
}

static void ip_event_sta_got_ip_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    static bool tcp_task = false;

    if (!tcp_task) {
        xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 8 * 1024, NULL, 5, NULL);
        xTaskCreate(light_refresh, "light_refresh", 4 * 1024, NULL, 5, NULL);
        tcp_task = true;
    }
}

static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

static void wifi_init(void)
{
    // Station
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ROUTER_SSID,
            .password = CONFIG_ROUTER_PASSWORD,
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Softap
    snprintf((char*)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", CONFIG_BRIDGE_SOFTAP_SSID);
    strlcpy((char*)wifi_config.ap.password, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(wifi_config.ap.password));
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_config);
}

void app_wifi_set_softap_info(void)
{
    char softap_ssid[32];
    char softap_psw[64];
    uint8_t softap_mac[6];
    size_t size = sizeof(softap_psw);
    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));

#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
    snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x", CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
    snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
    if (esp_mesh_lite_get_softap_ssid_from_nvs(softap_ssid, &size) != ESP_OK) {
        esp_mesh_lite_set_softap_ssid_to_nvs(softap_ssid);
    }
    if (esp_mesh_lite_get_softap_psw_from_nvs(softap_psw, &size) != ESP_OK) {
        esp_mesh_lite_set_softap_psw_to_nvs(CONFIG_BRIDGE_SOFTAP_PASSWORD);
    }
    esp_mesh_lite_set_softap_info(softap_ssid, CONFIG_BRIDGE_SOFTAP_PASSWORD);
}

void app_main()
{
    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);

    configure_led();

    esp_storage_init();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

    app_wifi_set_softap_info();

    esp_mesh_lite_start();

    /**
     * @breif Create handler
     */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_sta_got_ip_handler, NULL, NULL));

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_PERIOD_MS,
        true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
