/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include <memory.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

// Data pin for programming the strip
#define LED_STRIP_GPIO  4
// Numbers of the LED in the strip
#define NUM_LEDS 60
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static const char *TAG = "example";

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = NUM_LEDS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

#define M_TAU (M_PI * 2)

const uint32_t VALUE_SPEED_GPIO = GPIO_NUM_41;
const uint32_t BRIGHTNESS_CHANGE_GPIO = GPIO_NUM_42;
const uint32_t HUE_SPEED_GPIO = GPIO_NUM_40;

int32_t valuespeed = 0;
bool valuedir = 1;

int32_t brightness = 0x87;
bool brightnessdir = 1;

int32_t huespeed = 0;
bool huedir = 1;

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
//    ESP_LOGI(TAG, "Putting %lu into the queue...", gpio_num);
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void change_speed(int32_t *speed, bool *dir, int32_t low, int32_t high, uint32_t step) {
    if(*dir == 1) {
        *speed += step;

        if (*speed > high) {
            *speed = high;
            *dir = 0;
        }
    } else {
        *speed -= step;
        
        if (*speed <= low) {
            *speed = low;
            *dir = 1;
        }
    } 
}

static void gpio_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if(gpio_get_level(io_num) == 1) {
                ESP_LOGI(TAG, "Button on %lu pressed!", io_num);

                if (io_num == VALUE_SPEED_GPIO) {
                    change_speed(&valuespeed, &valuedir, -8, 8, 1);
                    ESP_LOGI(TAG, "Set valuespeed to %d", (int16_t) valuespeed);
                }
                else if (io_num == HUE_SPEED_GPIO) {
                    change_speed(&huespeed, &huedir, -8, 8, 1);
                    ESP_LOGI(TAG, "Set huespeed to %d", (int16_t) huespeed);
                }
                else if (io_num == BRIGHTNESS_CHANGE_GPIO) {
                    change_speed(&brightness, &brightnessdir, 0, 0xff, 0x0f);
                    ESP_LOGI(TAG, "Set brightness to 0x%x", (uint8_t) brightness);
                }
            }
        }
    }
}

void configure_pin(uint32_t pin) {
    gpio_config_t value_speed_button_io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_DEF_INPUT,
        .pull_down_en = 1,
        .pin_bit_mask = 1ULL << pin
    };

    gpio_config(&value_speed_button_io_conf);

    gpio_isr_handler_add(pin, gpio_isr_handler, (void*) pin);
}

void app_main(void)
{
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);

    configure_pin(VALUE_SPEED_GPIO);
    configure_pin(BRIGHTNESS_CHANGE_GPIO);
    configure_pin(HUE_SPEED_GPIO);

    led_strip_handle_t led_strip = configure_led();

    uint8_t *hues = calloc(NUM_LEDS, sizeof(uint8_t));
    double *values = calloc(NUM_LEDS, sizeof(double));

    int64_t huerotate = 0;
    int64_t valuerotate = 0;

    // Rainbow length = number of colors in a rainbow
    // Initialize the colors w a rainbow using HSV
    for(uint32_t i = 0; i < NUM_LEDS; i++) {
        hues[i] = (uint16_t) (0xFF * i / NUM_LEDS);

        double s = sin( (((double) i) * M_TAU) / ((double)NUM_LEDS) );

        values[i] = (s+1)/2; // (uint8_t) (s*s*0x2f);
        ESP_LOGI(TAG, "Pixel %lu got hue %d with value %f", i, hues[i], values[i]);
    }

    ESP_LOGI(TAG, "What?");
    while (1) {
        // Set each pixel to the HSV value
        for (int i = 0; i < NUM_LEDS; i++) {
            uint8_t hue = hues[(i + huerotate) % NUM_LEDS];
            uint8_t sat = 0xFF;
            uint8_t value = (uint8_t) 
                (
                    values[(i + (uint8_t) valuerotate) % NUM_LEDS] * 
                    (double) brightness
                );

            ESP_ERROR_CHECK(led_strip_set_pixel_hsv(led_strip, i, hue, sat, value));
        }
        /* Refresh the strip to send data */
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        huerotate += huespeed;
        valuerotate += valuespeed;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
