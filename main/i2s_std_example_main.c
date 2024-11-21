#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>  // Required for memset
//#include "esp_mac.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "driver/i2s.h"
#include "driver/gpio.h"
//#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define SAMPLE_RATE     44100
#define FREQUENCY_A4    440
#define AMPLITUDE       32767
#define BUFFER_SIZE     2048
#define I2S_PORT        I2S_NUM_0
#define PI              3.14159265358979323846f

#define SAMPLE_MAX      32767
#define SAMPLE_MIN      -32767

#define HIGH_PRIORITY configMAX_PRIORITIES - 1
#define EQUAL_PRIORITY tskIDLE_PRIORITY + 1



// Buffer to hold the generated audio data
int16_t *buffer;
QueueHandle_t i2s_event_queue;

enum WaveType {
    Sine,
    Square
};

enum BufferStatus {
    Empty,  // The buffer is just initialised and contains nothing.
    Ready,  // The buffer has fresh samples and is ready to be read.
    Spent   // The samples have been read, and the buffer needs new values.
};

struct Wave {
    enum WaveType type;
    float frequency;  // Frequency in Hz
    float amplitude;  // Amplitude (0.0 to 1.0)
    float phase;      // Current phase (0.0 to 2*PI)
    enum BufferStatus buffer_status;
    float* buffer;
};

struct WaveVector {
    struct Wave** data; // Pointer to array of Waves
    size_t size;       // Current number of elements
    size_t capacity;   // Allocated capacity
};

struct WaveVector waves;

void generateSineWaveSamplesStereo(struct Wave *wave);


// Function to initialize the waves structure
void initWaves() {
    waves.size = 0;
    waves.capacity = 4; // Initial capacity
    waves.data = malloc(waves.capacity * sizeof(struct Wave*));
    if (waves.data == NULL) {
        ESP_LOGE("initWaves", "Failed to allocate memory for waves data");
        abort();
    }
    ESP_LOGI("initWaves", "Waves initialized with capacity %zu", waves.capacity);
}

void generateSamples(struct Wave *wave) {
    char* TAG = "main:generateSamples";
    if (wave == NULL) {
        ESP_LOGE("main:generateSamples", "wave pointer is NULL");
        return;
    }
    //ESP_LOGV("main:generateSamples", "Top of function");
    switch (wave->type) {
        case Sine:
            generateSineWaveSamplesStereo(wave);
            wave->buffer_status = Ready; 
            break;

        case Square:
            break;    
    }
}

void generateSineWaveSamplesStereo(struct Wave *wave) {
    char* TAG = "main:generateSineWaveSamplesStereo";
    //ESP_LOGV("main:generateSineWaveSamplesStereo", "Top of function");
    
    if (!wave || !wave->buffer) {
        return; // Handle null pointers gracefully
    }

    // Stereo audio: each frame has two samples (left and right channel)
    size_t frame_count = BUFFER_SIZE / 2; // Each frame has two samples

    float phase = 0.0f;
    size_t tmp = 0;
    for (size_t i = 0; i < frame_count; i++) {
        //ESP_LOGV(TAG, "Start of loop");
        // Compute the sample value for the sine wave
        //ESP_LOGI(TAG, "(float)i=%f i=%d (float)SAMPLE_RATE=%f  SAMPLE_RATE=%d wave->phase=%f", (float)i, i, (float)SAMPLE_RATE, SAMPLE_RATE,wave->phase);
        float t = ((float)i / (float)SAMPLE_RATE) + wave->phase; // Time of the sample
        float value = wave->amplitude * sinf(2.0f * PI * wave->frequency * t);
        phase = t;

        //ESP_LOGV(TAG, "Before wave->buffer[2 * i] = value; - 2 * i=%d", 2 * i);
        wave->buffer[2 * i] = value;             // Left channel
        //ESP_LOGV(TAG, "Before wave->buffer[2 * i + 1] = value; - 2 * i=%d", 2 * i + 1);
        wave->buffer[2 * i + 1] = value;         // Right channel   

        //ESP_LOGI(TAG, "%d %f %f %f  - amp=%f, wave->frequency=%f, t=%f, wave->phase=%f, 2 * i=%d, 2 * i + 1=%d, frame_count=%d", i, wave->buffer[2 * i], wave->buffer[2 * i + 1], value, wave->amplitude, wave->frequency, t, wave->phase, 2 * i, 2 * i + 1, frame_count);
        //ESP_LOGI(TAG, "%d %f", i, value);
        tmp = i;
    }
    /*ESP_LOGI(TAG, "After loop - ==================== %d =======================", tmp);
    for (size_t i = 0; i < frame_count; i++){
        ESP_LOGI(TAG, "%d %f", i, wave->buffer[2 * i]);
    }*/
    
    // Update the phase to avoid discontinuity
    wave->phase = phase;
    
}

void addWave(enum WaveType waveType, float frequency) {
    char* TAG = "main:addWave";
    
    struct Wave* wave = malloc(sizeof(struct Wave));
    if (wave == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for Wave");
        exit(EXIT_FAILURE);
    }

    wave->type = waveType;
    wave->frequency = frequency;
    wave->amplitude = 1;
    wave->phase = ((float)rand() / (float)RAND_MAX) * (2.0f * M_PI);
    wave->buffer_status = Empty;
    wave->buffer = (float *)malloc(BUFFER_SIZE * sizeof(float));;

    // Resize waves if needed
    if (waves.size == waves.capacity) {
        waves.capacity++;
        ESP_LOGI(TAG, "Free heap before realloc: %d, we are addng %d", heap_caps_get_free_size(MALLOC_CAP_8BIT), sizeof(struct Wave*));
        waves.data = realloc(waves.data, waves.capacity * sizeof(struct Wave*));
        if (!waves.data) {
            ESP_LOGE(TAG, "Failed to reallocate memory");
            exit(EXIT_FAILURE);
        }
    }
    waves.data[waves.size] = wave;
    waves.size++;

    generateSamples(wave);
}

void sumWaves(int16_t *buffer) {
    char* TAG = "main:sumWaves";
    // Reset the buffer
    memset(buffer, 0, BUFFER_SIZE * sizeof(int16_t));

    float* floatBuffer = (float*)malloc(BUFFER_SIZE * sizeof(float));
    if (floatBuffer == NULL) {
        ESP_LOGE(TAG, "Something went wrong when allocating an array.");
        return;
    }

    float max_sample = 1.0f;
    float min_sample = -1.0f;
    for(size_t j = 0; j < BUFFER_SIZE; j++) {
        float sum = 0;
        for(size_t k = 0; k < waves.size; k++) {
            sum += waves.data[k]->buffer[j];
            floatBuffer[j] = sum;
        }
        
        if (sum > max_sample) {
            max_sample = sum;
        }
        if (sum < min_sample) {
            min_sample = sum;
        }
        //ESP_LOGI(TAG, "%d %d %f %f", j, waves.size, waves.data[0]->buffer[j], floatBuffer[j]);
    }

    float range = max_sample - min_sample;
    if (range == 0.0f) {
        range = 1.0f; // Avoid division by zero
    }
    float scale = (float)SAMPLE_MAX / range; // Normalize to the range of int16 (-32768 to 32767)
    //ESP_LOGI(TAG, "=======================================================================");
    // Normalize and convert each sample
    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        // Normalize to [-1, 1] and scale to int16 range
        //(-1.655035 + -1.906246 / 3.462053) * (32767- -32767) - 32
        float normalisedSample = ((floatBuffer[i] - min_sample) / range) * (float)(SAMPLE_MAX-SAMPLE_MIN) + (float)SAMPLE_MIN;
        //float normalisedSample = (floatBuffer[i] - min_sample) * scale - (float)SAMPLE_MAX;
        buffer[i] = (int16_t)roundf(normalisedSample);

        //ESP_LOGI(TAG, "min_sample=%f, max_sample=%f, range=%f, scale=%f, %d %f %d", min_sample, max_sample, range, scale, i, floatBuffer[i], buffer[i]);
        // Clip if necessary, but warn because it shouldn't be possible.
        if (buffer[i] > SAMPLE_MAX) {
            ESP_LOGW(TAG, "A value of over SAMPLE_MAX has been written to the buffer. This should not be possible. i=%d, buffer[i]=%d, normalisedSample=%f", i, buffer[i], normalisedSample);
            buffer[i] = SAMPLE_MAX;
        } else if (buffer[i] < SAMPLE_MIN) {
            ESP_LOGW(TAG, "A value of under SAMPLE_MIN has been written to the buffer. This should not be possible. i=%d, buffer[i]=%d, normalisedSample=%f", i, buffer[i], normalisedSample);
            buffer[i] = SAMPLE_MIN;
        }
    }
    
    free(floatBuffer); 
    
}




void setup_i2s() {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE/2, // Because of Stereo
        .use_apll = false,
        .tx_desc_auto_clear = true
    };


    i2s_pin_config_t pin_config = {
        .bck_io_num = 16,   // Bit clock
        .ws_io_num = 17,    // Word select (LR clock)
        .data_out_num = 26, // Data output
        .data_in_num = I2S_PIN_NO_CHANGE  // Not used
    };

    // Install the I2S driver and configure it with event queue
    i2s_driver_install(I2S_PORT, &i2s_config, 10, &i2s_event_queue);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_set_sample_rates(I2S_PORT, SAMPLE_RATE);
    i2s_zero_dma_buffer(I2S_PORT);
}

// Task to handle I2S events and refill buffer
void i2s_event_task(void *param) {
    char* TAG = "main:i2s_event_task";
    i2s_event_t i2s_event;
    while (1) {
        // Wait for the I2S event from the queue
        if (xQueueReceive(i2s_event_queue, &i2s_event, portMAX_DELAY) == pdTRUE) {
            if (i2s_event.type == I2S_EVENT_TX_DONE) {
                ESP_LOGI(TAG, "i2s_event.type == I2S_EVENT_TX_DONE");
                vTaskPrioritySet(xTaskGetCurrentTaskHandle(), HIGH_PRIORITY);
                // Refill the buffer here upon receiving TX_DONE
                //generate_square_wave_a4_stereo(buffer, BUFFER_SIZE);
                sumWaves(buffer);

                // Write the new data to I2S
                size_t bytes_written;
                i2s_write(I2S_PORT, buffer, BUFFER_SIZE * sizeof(int16_t), &bytes_written, portMAX_DELAY);
                // If all buffers were written as planned, we set the buffer_status to Spent so the main loop
                // know that new data needs to be written.
                if (bytes_written == BUFFER_SIZE * sizeof(int16_t)) {
                    //ESP_LOGV(TAG, "bytes_written == BUFFER_SIZE = true");
                    for (size_t i = 0; i < waves.size; i++) { 
                        waves.data[i]->buffer_status = Spent;
                        //ESP_LOGV(TAG, "Just after waves.data[i]->buffer_status = Spent....i=%d, waves.data[i]->buffer_status=%d", i, waves.data[i]->buffer_status);
                    } 
                } else {
                    ESP_LOGW(TAG, "WARNING: not all data that should have been written to I2S was. bytes_written=%d, BUFFER_SIZE=%d", bytes_written, BUFFER_SIZE);
                }
                vTaskPrioritySet(xTaskGetCurrentTaskHandle(), EQUAL_PRIORITY);
            }
        }
        taskYIELD();
    }
}

int write_counter = 0;
int idle_counter = 0;

void checkI2S() {
    char* TAG = "main:checkI2S";
    i2s_event_t i2s_event;
    if (xQueueReceive(i2s_event_queue, &i2s_event, portMAX_DELAY) == pdTRUE) {
        if (i2s_event.type == I2S_EVENT_TX_DONE) {
            write_counter++;
            //ESP_LOGI(TAG, "i2s_event.type == I2S_EVENT_TX_DONE");
            // Refill the buffer here upon receiving TX_DONE
            sumWaves(buffer);

            // Write the new data to I2S
            size_t bytes_written;
            i2s_write(I2S_PORT, buffer, BUFFER_SIZE * sizeof(int16_t), &bytes_written, portMAX_DELAY);
            /*for (int i = 0; i < BUFFER_SIZE; i += 2) {
                ESP_LOGV("Dump", "%d %d", i, buffer[i]);
            }*/
            // If all buffers were written as planned, we set the buffer_status to Spent so the main loop
            // know that new data needs to be written.
            if (bytes_written == BUFFER_SIZE * sizeof(int16_t)) {
                //ESP_LOGV(TAG, "bytes_written == BUFFER_SIZE = true");
                //ESP_LOGV(TAG, "%d - %d - %d - %d - %d - %d - %d - %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
                for (size_t i = 0; i < waves.size; i++) { 
                    waves.data[i]->buffer_status = Spent;
                    //ESP_LOGV(TAG, "Just after waves.data[i]->buffer_status = Spent....i=%d, waves.data[i]->buffer_status=%d", i, waves.data[i]->buffer_status);
                } 
            } else {
                ESP_LOGW(TAG, "WARNING: not all data that should have been written to I2S was. bytes_written=%d, BUFFER_SIZE=%d", bytes_written, BUFFER_SIZE);
            }
        } else {
            idle_counter++;
            //ESP_LOGW(TAG, "This is actually good, we should arrive here if there is time to spare. And there shoudl be time to spare.");
        }
    }
}

void app_main(void) {
    char* TAG = "main:app_main";
    ESP_LOGI(TAG, "Start of app_main");
    //vTaskPrioritySet(xTaskGetCurrentTaskHandle(), EQUAL_PRIORITY);

    initWaves();

    //gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);


    // Initialize the I2S interface
    setup_i2s();

    // Allocate memory for the buffer on the heap
    buffer = (int16_t *)malloc(BUFFER_SIZE * sizeof(int16_t));
    if (buffer == NULL) {
        printf("Failed to allocate memory for buffer\n");
        return;
    }

    addWave(Sine, 440.0f);
    //addWave(Sine, 880.0f);
    //addWave(Sine, 220.0f);
    ESP_LOGI(TAG, "sizeof(waves.data[0])=%d", sizeof(waves.data[0]));
    ESP_LOGI(TAG, "sizeof(waves.data[0]->type)=%d", sizeof(waves.data[0]->type));
    ESP_LOGI(TAG, "sizeof(waves.data[0]->frequency)=%d", sizeof(waves.data[0]->frequency));
    ESP_LOGI(TAG, "sizeof(waves.data[0]->amplitude)=%d", sizeof(waves.data[0]->amplitude));
    ESP_LOGI(TAG, "sizeof(waves.data[0]->phase)=%d", sizeof(waves.data[0]->phase));
    ESP_LOGI(TAG, "sizeof(waves.data[0]->buffer_status)=%d", sizeof(waves.data[0]->buffer_status));
    ESP_LOGI(TAG, "sizeof(waves.data[0]->buffer)=%d", sizeof(waves.data[0]->buffer));

    // Generate the initial waveform in the buffer
    //generate_square_wave_a4_stereo(buffer, BUFFER_SIZE);

    // Write the initial buffer to I2S to start playback
    size_t bytes_written;
    i2s_write(I2S_PORT, buffer, BUFFER_SIZE * sizeof(int16_t), &bytes_written, portMAX_DELAY);

    //ESP_LOGI("Stack Check", "Free stack size before creating I2S Event Task: %d", uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()));


    //ESP_LOGV(TAG, "Before creating task");
    // Create a FreeRTOS task to handle I2S events. We give the task the same priority as the main loop.
    //xTaskCreate(i2s_event_task, "I2S Event Task", 2048, NULL, EQUAL_PRIORITY, NULL);
    //ESP_LOGV(TAG, "After creating task");

    while (1) {
        //ESP_LOGV(TAG, "Top of main loop. ");
        //ESP_LOGI("Stack Check", "Free stack size in main loop: %d", uxTaskGetStackHighWaterMark(xTaskGetCurrentTaskHandle()));
        for (size_t i = 0; i < waves.size; i++) { 
            //ESP_LOGV(TAG, "Going through the waves ");
            //vTaskDelay(pdMS_TO_TICKS(1000));
            if (waves.data[i]->buffer_status == Spent) {  
                //vTaskPrioritySet(xTaskGetCurrentTaskHandle(), HIGH_PRIORITY);
                //ESP_LOGV(TAG, "i=%d", i);   
                //ESP_LOGI("Stack Check", "Free stack size before generateSamples(): %d", uxTaskGetStackHighWaterMark(app_main));    
                generateSamples(waves.data[i]);
                //ESP_LOGI(TAG, "Found a Spent buffer. waves.data[i]->buffer_status=%d", waves.data[i]->buffer_status);
                //vTaskDelay(pdMS_TO_TICKS(1000));
                //vTaskPrioritySet(xTaskGetCurrentTaskHandle(), EQUAL_PRIORITY);
            }
            
        } 

        checkI2S();

        //vTaskDelay(pdMS_TO_TICKS(100));  // Adjust delay as needed
        taskYIELD();

        if (idle_counter+write_counter > 1000) {
            ESP_LOGW(TAG, "idle_counter=%d, write_counter=%d", idle_counter, write_counter);
        }
    }

    // Free the buffer if you ever exit the loop
    free(buffer);
}
