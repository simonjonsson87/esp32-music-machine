#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "esp_log.h"

static const char *TAG = "BLINK_APP";  // Tag for log messages

#define BLINK_GPIO GPIO_NUM_2

// Function to initialize GPIO
void init_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLINK_GPIO),  // Select GPIO2
        .mode = GPIO_MODE_OUTPUT,             // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,    // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,// Disable pull-down
        .intr_type = GPIO_INTR_DISABLE        // Disable interrupts
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "GPIO %d initialized as output", BLINK_GPIO);
}

// Task to blink LED
void blink_task(void *param) {
    while (1) {
        gpio_set_level(BLINK_GPIO, 1);  // Turn LED on
        ESP_LOGD(TAG, "GPIO %d set to HIGH", BLINK_GPIO);  // Debug message
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 500ms

        gpio_set_level(BLINK_GPIO, 0);  // Turn LED off
        ESP_LOGD(TAG, "GPIO %d set to LOW", BLINK_GPIO);  // Debug message
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 500ms
    }
}

void app_main(void) {
    // Set the log level for this tag to DEBUG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    init_gpio();                 // Initialize GPIO
    xTaskCreate(blink_task,      // Task function
                "blink_task",    // Task name
                2048,            // Stack size in bytes
                NULL,            // Parameter to pass
                5,               // Task priority
                NULL);           // Task handle
}
