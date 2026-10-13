#ifndef RUN_PAPER_TEST
/**
 * @file halTest.cpp
 * @author Alessandro Monticelli (alessandr.monticell4@studio.unibo.it)
 * @brief Test sketches for SX1262 HAL created for the Heltec LoRa 32v3 modules.
 * @version 0.1
 * @date 2026-02-06
 *
 * @copyright Copyright (c) 2026
 *
 */

/*
# define HAL_TEST_TRANS // Uncomment to test HAL for the Transmitter module.
*/

/*
# define HAL_TEST_RECV // Uncomment to test HAL for the Receiver module.
*/

#ifdef HAL_TEST_TRANS
#include <RadioLib.h>

#include "EspHal.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HalTest";

// 1. Instantiate the HAL with SPI pins (SCK, MISO, MOSI)
EspHal hal_inst(HELTEC_LORA_SCK, HELTEC_LORA_MISO, HELTEC_LORA_MOSI);
EspHal *hal = &hal_inst;

// 2. Instantiate the SX1262 module using the HAL and control pins (NSS, DIO1, RST, BUSY)
Module radioModule_inst(hal, HELTEC_LORA_NSS, HELTEC_LORA_DIO1, HELTEC_LORA_RST, HELTEC_LORA_BUSY);
SX1262 radio(&radioModule_inst);

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "=== HAL TEST STARTED ===");

  // 3. CRITICAL STEP: Turn Vext Power ON (GPIO 36)
  // Without this, the LoRa chip receives no power.
  gpio_reset_pin(HELTEC_POWER_CTRL);
  gpio_set_direction(HELTEC_POWER_CTRL, GPIO_MODE_OUTPUT);
  gpio_set_level(HELTEC_POWER_CTRL, 0);  // LOW = ON
  vTaskDelay(pdMS_TO_TICKS(100));        // Wait for voltage to stabilize

  // 4. RadioLib Initialization
  // If the HAL works (SPI ok, GPIO ok), this method returns 0 (RADIOLIB_ERR_NONE)
  ESP_LOGI(TAG, "Initializing SX1262...");
  int state = radio.begin(868.0);  // Frequency 868.0 MHz

  if (state == RADIOLIB_ERR_NONE)
  {
    ESP_LOGI(TAG, "SUCCESS! Radio initialized successfully.");
  }
  else
  {
    ESP_LOGE(TAG, "FAILED. Error code: %d", state);
    // If it fails here, check the pins in EspHal.h or solder joints
    while (true)
      vTaskDelay(1000);
  }

  // 5. Transmission test loop
  while (true)
  {
    ESP_LOGI(TAG, "Attempting raw packet transmission...");

    // Send a simple string (without custom protocol)
    state = radio.transmit("Test HAL OK!");

    if (state == RADIOLIB_ERR_NONE)
    {
      ESP_LOGI(TAG, "TX Completed successfully!");
    }
    else
    {
      ESP_LOGE(TAG, "TX Error: %d", state);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
#endif

#ifdef HAL_TEST_RECV
#include <RadioLib.h>

#include <cstring>

#include "EspHal.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HalTestRX";

// 1. Instantiate the HAL and the Module
EspHal hal_inst(HELTEC_LORA_SCK, HELTEC_LORA_MISO, HELTEC_LORA_MOSI);
EspHal *hal = &hal_inst;
Module radioModule_inst(hal, HELTEC_LORA_NSS, HELTEC_LORA_DIO1, HELTEC_LORA_RST, HELTEC_LORA_BUSY);
SX1262 radio(&radioModule_inst);

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "=== RECEIVER TEST (BYTE ARRAY) ===");

  // 2. Vext Power ON
  gpio_reset_pin(HELTEC_POWER_CTRL);
  gpio_set_direction(HELTEC_POWER_CTRL, GPIO_MODE_OUTPUT);
  gpio_set_level(HELTEC_POWER_CTRL, 0);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 3. Manual HAL Init
  hal->init();

  // 4. Start Radio
  int state = radio.begin(868.0);
  if (state == RADIOLIB_ERR_NONE)
  {
    ESP_LOGI(TAG, "Radio Initialized! Waiting...");
  }
  else
  {
    ESP_LOGE(TAG, "Init Failed: %d", state);
    while (true)
      vTaskDelay(1000);
  }

  // 5. Receive Loop
  uint8_t rxBuffer[256];  // Static buffer for raw data

  while (true)
  {
    // receive() in this overload accepts (buffer, max_length)
    state = radio.receive(rxBuffer, sizeof(rxBuffer));

    if (state == RADIOLIB_ERR_NONE)
    {
      // Retrieve the actual length of the received packet
      size_t len = radio.getPacketLength();

      ESP_LOGI(TAG, "PACKET RECEIVED! (Len: %d)", (int)len);

      // Print the content as a string (if text) or hex
      // Note: rxBuffer is not necessarily null-terminated, so we use %.*s format
      ESP_LOGI(TAG, "Data: %.*s", (int)len, rxBuffer);

      ESP_LOGI(TAG, "RSSI: %.2f dBm", radio.getRSSI());
      ESP_LOGI(TAG, "SNR:  %.2f dB", radio.getSNR());
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT)
    {
      // Timeout: normal, retry
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
      ESP_LOGW(TAG, "CRC Error");
    }
    else
    {
      ESP_LOGE(TAG, "RX Error: %d", state);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif
#endif  // RUN_PAPER_TEST
