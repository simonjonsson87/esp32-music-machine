#include <stdio.h>
#include <math.h>
#include <stdlib.h>  // for malloc and free
#include "driver/i2s.h"

#define SAMPLE_RATE     44100
#define FREQUENCY_A4    440
#define AMPLITUDE       32767
#define BUFFER_SIZE     1024  // Adjust as needed; must be even for stereo
#define I2S_PORT        I2S_NUM_0

void generate_square_wave_a4_stereo(int16_t *buffer, int buffer_size) {
    int period_samples = SAMPLE_RATE / FREQUENCY_A4;
    int half_period = period_samples / 2;

    for (int i = 0; i < buffer_size / 2; i++) {  // Stereo output
        int16_t sample_value = (i % period_samples) < half_period ? AMPLITUDE : -AMPLITUDE;

        buffer[i * 2]     = sample_value;  // Left channel
        buffer[i * 2 + 1] = sample_value;  // Right channel
    }
}

void setup_i2s() {
    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,  // Default interrupt priority
        .dma_buf_count = 8,     // Number of DMA buffers
        .dma_buf_len = BUFFER_SIZE,  // Size of each DMA buffer in samples
        .use_apll = false,
        .tx_desc_auto_clear = true  // Auto clear tx descriptor on underflow
    };

    // Pin configuration for the PCM5102 DAC (adjust as needed)
    i2s_pin_config_t pin_config = {
        .bck_io_num = 16,   // Bit clock
        .ws_io_num = 17,    // Word select (LR clock)
        .data_out_num = 26, // Data output
        .data_in_num = I2S_PIN_NO_CHANGE  // Not used
    };

    // Install and start I2S driver
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);  // Clear the DMA buffer
}

void app_main(void) {
    // Initialize the I2S interface
    setup_i2s();

    // Dynamically allocate memory for the buffer on the heap
    int16_t *buffer = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    if (buffer == NULL) {
        printf("Failed to allocate memory for buffer\n");
        return;
    }

    // Generate a square wave and fill the buffer
    generate_square_wave_a4_stereo(buffer, BUFFER_SIZE);

    // Continuously send the buffer to I2S
    while (1) {
        size_t bytes_written;
        i2s_write(I2S_PORT, buffer, BUFFER_SIZE * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    }

    // Free the allocated memory (if we were to exit this loop)
    free(buffer);
}

