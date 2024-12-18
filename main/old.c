/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"

void generate_square_wave_a4_stereo(int16_t *w_buf, int buffer_size);

/* Set 1 to allocate rx & tx channels in duplex mode on a same I2S controller, they will share the BCLK and WS signal
 * Set 0 to allocate rx & tx channels in simplex mode, these two channels will be totally separated,
 * Specifically, due to the hardware limitation, the simplex rx & tx channels can't be registered on the same controllers on ESP32 and ESP32-S2,
 * and ESP32-S2 has only one I2S controller, so it can't allocate two simplex channels */
#define EXAMPLE_I2S_DUPLEX_MODE         0

#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_STD_BCLK_IO1        GPIO_NUM_16      // I2S bit clock io number
#define EXAMPLE_STD_WS_IO1          GPIO_NUM_17      // I2S word select io number
#define EXAMPLE_STD_DOUT_IO1        GPIO_NUM_26 //26     // I2S data out io number
#define EXAMPLE_STD_DIN_IO1         GPIO_NUM_19     // I2S data in io number
#if !EXAMPLE_I2S_DUPLEX_MODE
#define EXAMPLE_STD_BCLK_IO2    GPIO_NUM_16     // I2S bit clock io number
#define EXAMPLE_STD_WS_IO2      GPIO_NUM_17     // I2S word select io number
#define EXAMPLE_STD_DOUT_IO2    GPIO_NUM_26     // I2S data out io number
#define EXAMPLE_STD_DIN_IO2     GPIO_NUM_19     // I2S data in io number
#endif
#else
#define EXAMPLE_STD_BCLK_IO1        GPIO_NUM_2      // I2S bit clock io number
#define EXAMPLE_STD_WS_IO1          GPIO_NUM_3      // I2S word select io number
#define EXAMPLE_STD_DOUT_IO1        GPIO_NUM_4      // I2S data out io number
#define EXAMPLE_STD_DIN_IO1         GPIO_NUM_5      // I2S data in io number
#if !EXAMPLE_I2S_DUPLEX_MODE
#define EXAMPLE_STD_BCLK_IO2    GPIO_NUM_6      // I2S bit clock io number
#define EXAMPLE_STD_WS_IO2      GPIO_NUM_7      // I2S word select io number
#define EXAMPLE_STD_DOUT_IO2    GPIO_NUM_8      // I2S data out io number
#define EXAMPLE_STD_DIN_IO2     GPIO_NUM_9      // I2S data in io number
#endif
#endif

#define EXAMPLE_BUFF_SIZE               2048

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler

static void i2s_example_write_task(void *args)
{
    /*
    uint8_t *w_buf = (uint8_t *)calloc(1, EXAMPLE_BUFF_SIZE);
    assert(w_buf); // Check if w_buf allocation success

    // Assign w_buf 
    for (int i = 0; i < EXAMPLE_BUFF_SIZE; i += 8) {
        w_buf[i]     = 0x12;
        w_buf[i + 1] = 0x34;
        w_buf[i + 2] = 0x56;
        w_buf[i + 3] = 0x78;
        w_buf[i + 4] = 0x9A;
        w_buf[i + 5] = 0xBC;
        w_buf[i + 6] = 0xDE;
        w_buf[i + 7] = 0xF0;
    }
    */

    uint16_t *w_buf = (uint16_t *)calloc(1, EXAMPLE_BUFF_SIZE);
    assert(w_buf); // Check if w_buf allocation success
    generate_square_wave_a4_stereo(*w_buf, EXAMPLE_BUFF_SIZE);


    size_t w_bytes = EXAMPLE_BUFF_SIZE;

    /* (Optional) Preload the data before enabling the TX channel, so that the valid data can be transmitted immediately */
    while (w_bytes == EXAMPLE_BUFF_SIZE) {
        /* Here we load the target buffer repeatedly, until all the DMA buffers are preloaded */
        ESP_ERROR_CHECK(i2s_channel_preload_data(tx_chan, w_buf, EXAMPLE_BUFF_SIZE, &w_bytes));
    }

    /* Enable the TX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    while (1) {
        /* Write i2s data */
        if (i2s_channel_write(tx_chan, w_buf, EXAMPLE_BUFF_SIZE, &w_bytes, 1000) == ESP_OK) {
            printf("Write Task: i2s write %d bytes\n", w_bytes);
        } else {
            printf("Write Task: i2s write failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); //
    }
    free(w_buf);
    vTaskDelete(NULL);
}

static void i2s_example_init_std_simplex(void)
{
    /* Setp 1: Determine the I2S channel configuration and allocate two channels one by one
     * The default configuration can be generated by the helper macro,
     * it only requires the I2S controller id and I2S role
     * The tx and rx channels here are registered on different I2S controller,
     * Except ESP32 and ESP32-S2, others allow to register two separate tx & rx channels on a same controller */
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));
   

    /* Step 2: Setting the configurations of standard mode and initialize each channels one by one
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100), // 16000
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = EXAMPLE_STD_BCLK_IO1,
            .ws   = EXAMPLE_STD_WS_IO1,
            .dout = EXAMPLE_STD_DOUT_IO1,
            .din  = EXAMPLE_STD_DIN_IO1,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));

}

void app_main(void)
{

    i2s_example_init_std_simplex();


    /* Step 3: Create writing task, enable and start the channel */
    xTaskCreate(i2s_example_write_task, "i2s_example_write_task", 4096, NULL, 5, NULL);
}




#define SAMPLE_RATE 44100
#define FREQUENCY_A4 440
#define AMPLITUDE 32767  // Max amplitude for 16-bit audio
//#define EXAMPLE_BUFF_SIZE 1024 
void generate_square_wave_a4_stereo(int16_t *w_buf, int buffer_size) {
    // Calculate the total number of samples in one period of the square wave
    int period_samples = SAMPLE_RATE / FREQUENCY_A4;
    int half_period = period_samples / 2;

    for (int i = 0; i < buffer_size / 2; i++) {  // Divide by 2 because we have stereo (two channels)
        // Determine if we are in the high or low part of the square wave
        int16_t sample_value = (i % period_samples) < half_period ? AMPLITUDE : -AMPLITUDE;

        // Set the same value for both left and right channels for stereo output
        w_buf[i * 2]     = sample_value;  // Left channel
        w_buf[i * 2 + 1] = sample_value;  // Right channel
    }
}

