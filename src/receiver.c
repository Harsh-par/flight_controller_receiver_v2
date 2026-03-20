#include "receiver.h"

void receiver_init(){
    esp_err_t error = nvs_flash_init();

    if(error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_configuration = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_configuration));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /*
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t ap_configuration = {
        .ap = {
            .ssid           = RX_SSID,
            .ssid_len       = 0,
            .password       = RX_PASSWORD,
            .channel        = ESP_NOW_CHANNEL,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .max_connection = 4,
            .ssid_hidden    = 0
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_configuration));
    ESP_ERROR_CHECK(esp_wifi_start());
    */


    ESP_ERROR_CHECK(esp_now_init());

    esp_now_register_recv_cb(receiver_callback);

    //init the pcb led (used to show when armed)
    gpio_config_t gpio_configuration = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << PIN_INDICATOR_LED),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE
    };

    gpio_config(&gpio_configuration);
}

void receiver_callback(const esp_now_recv_info_t *receive_info, const uint8_t *data, int data_length){
    if(data_length != sizeof(controller_data_t)) return;

    memcpy( (void*)&transmitter, data, sizeof(controller_data_t) );

    time_us_last_packet = esp_timer_get_time();
}

void receiver_check_connection(bool* receiver_connected, bool* receiver_armed, int64_t time_us_current){
    if(time_us_current - time_us_last_packet > PACKET_TIMEOUT_US){
        *receiver_connected  = false;
        *receiver_armed      = false;
    }
    else *receiver_connected = true;
}

void receiver_check_arming(bool* receiver_armed, volatile controller_data_t* transmitter, motor_t* motor_a, motor_t* motor_b, motor_t* motor_c, motor_t* motor_d){
    static int64_t time_us_current  = 0;
    static int64_t time_us_previous = 0;

    time_us_current = esp_timer_get_time();

    if(time_us_current - time_us_previous > BUTTON_DEBOUNCE_US && transmitter->button_m)
    {
        *receiver_armed = !(*receiver_armed);
        
        time_us_previous = time_us_current;
    }

    if(!(*receiver_armed))
    {
        motor_stop(motor_a);
        motor_stop(motor_b);
        motor_stop(motor_c);
        motor_stop(motor_d);
        gpio_set_level(PIN_INDICATOR_LED, false);

    }
    else gpio_set_level(PIN_INDICATOR_LED, true);
}

void receiver_check_voltage(bool* receiver_voltage, bool* receiver_armed, battery_manager_t* battery_manager){
    #ifdef ENABLE_BATTERY_MANAGER
        if(battery_manager->battery_status == BATTERY_STATUS_CRITICAL)
        {
            *receiver_voltage = false;
            *receiver_armed   = false;
        }
        else *receiver_voltage = true;
    #else        
        *receiver_voltage = true;
    #endif
}