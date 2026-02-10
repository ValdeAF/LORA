#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "sx127x.h" 

#define SCK_GPIO  5
#define MISO_GPIO 19
#define MOSI_GPIO 27
#define CS_GPIO   18
#define RST_GPIO  14
#define DIO0_GPIO 26

static const char *TAG = "LORA";

// 1. Your library requires manual reset (it doesn't handle GPIOs inside the struct)
static void lora_reset(void) {
    gpio_set_direction((gpio_num_t)RST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level((gpio_num_t)RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lora_rx_callback(void *ctx, uint8_t *data, uint16_t length) {
    (void)ctx;
    char text[256];
    size_t copy_len = length < (sizeof(text) - 1) ? length : (sizeof(text) - 1);

    memcpy(text, data, copy_len);
    text[copy_len] = '\0';

    ESP_LOGI(TAG, "RX %u bytes: %s", (unsigned)length, text);
}

static void lora_tx_callback(void *ctx) {
    (void)ctx;
    ESP_LOGI(TAG, "TX done");
}

static void lora_receive_loop(sx127x *device) {
    ESP_LOGI(TAG, "Starting RX loop");
    gpio_set_direction((gpio_num_t)DIO0_GPIO, GPIO_MODE_INPUT);
    sx127x_rx_set_callback(lora_rx_callback, device, device);
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_RX_CONT, SX127X_MODULATION_LORA, device));

    while (1) {
        if (gpio_get_level((gpio_num_t)DIO0_GPIO)) {
            sx127x_handle_interrupt(device);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void lora_send_loop(sx127x *device) {
    static const char *message = "hello";

    ESP_LOGI(TAG, "Starting TX loop");
    gpio_set_direction((gpio_num_t)DIO0_GPIO, GPIO_MODE_INPUT);
    sx127x_tx_set_callback(lora_tx_callback, device, device);

    while (1) {
        ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission((const uint8_t *)message,
                                                            (uint8_t)(strlen(message) + 1),
                                                            device));
        ESP_ERROR_CHECK(sx127x_set_opmod(SX127X_MODE_TX, SX127X_MODULATION_LORA, device));

        while (!gpio_get_level((gpio_num_t)DIO0_GPIO)) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        sx127x_handle_interrupt(device);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing SPI Bus...");

    // 2. Initialize the SPI Bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI_GPIO,
        .miso_io_num = MISO_GPIO,
        .sclk_io_num = SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, 1));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8 * 1000 * 1000, 
        .mode = 0,
        .spics_io_num = CS_GPIO,
        .queue_size = 7,
        .command_bits = 0, 
        .address_bits = 8,
        .dummy_bits = 0
    };
    
    spi_device_handle_t spi_handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle));

    // 3. Reset the hardware manually
    lora_reset();

    // 4. Create the device (Your library takes the SPI handle DIRECTLY)
    sx127x device; 
    ESP_ERROR_CHECK(sx127x_create(spi_handle, &device));

    // 5. Configure Radio (Using CORRECT CAPS: SX127X_...)
    ESP_LOGI(TAG, "Configuring Radio...");
    
    // Note: The 'X' must be CAPITAL in your library constants
    sx127x_set_opmod(SX127X_MODE_SLEEP, SX127X_MODULATION_LORA, &device); // Sleep to config
    sx127x_set_frequency(868000000, &device);
    sx127x_lora_set_bandwidth(SX127X_BW_125000, &device);
    sx127x_lora_set_spreading_factor(SX127X_SF_7, &device);
    sx127x_lora_set_syncword(0x12, &device);
    sx127x_set_preamble_length(8, &device);
    sx127x_lora_set_implicit_header(NULL, &device);
    sx127x_lora_reset_fifo(&device);
    sx127x_set_opmod(SX127X_MODE_STANDBY, SX127X_MODULATION_LORA, &device); // Wake up

    ESP_LOGI(TAG, "SX127x initialized successfully!");

    sx127x_tx_header_t header = {
        .enable_crc = true,
        .coding_rate = SX127X_CR_4_5
    };
    sx127x_lora_tx_set_explicit_header(&header, &device);

    // Pick one loop. Comment out the one you don't want.
    lora_receive_loop(&device);
    //lora_send_loop(&device);
}