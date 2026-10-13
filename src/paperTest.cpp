#ifdef RUN_PAPER_TEST
#include <RadioLib.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "EspHal.hpp"
#include "LoRaProtocol.hpp"
#include "Ssd1306.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PaperTest";
static const uint8_t PAPER_TEST_ADDR_TX = 0x01;
static const uint8_t PAPER_TEST_ADDR_RX = 0x02;

// Global Hardware Instances
EspHal hal_inst(HELTEC_LORA_SCK, HELTEC_LORA_MISO, HELTEC_LORA_MOSI);
EspHal *hal = &hal_inst;
Module radioModule_inst(hal, HELTEC_LORA_NSS, HELTEC_LORA_DIO1, HELTEC_LORA_RST, HELTEC_LORA_BUSY);
SX1262 radio(&radioModule_inst);
LoRaProtocol protocol_inst(&radio, hal, HELTEC_LORA_DIO1);
LoRaProtocol *protocol = &protocol_inst;
Ssd1306 oled;

static esp_err_t oledErr = ESP_FAIL;

std::vector<size_t> currentPayloadSizes = {1, 246, 247, 492, 1000, 2000, 5000};
int currentIterations = 10;
static int simulatedDropRate = 0;

void updateOled(const std::string &title, const std::string &status)
{
  if (oledErr != ESP_OK)
    return;
  oled.clear();
  oled.print(0, 0, " LAMP PAPER TEST ");
  oled.print(2, 0, title.c_str());
  oled.print(4, 0, status.c_str());
  oled.update();
}

static std::vector<uint8_t> generatePatternPayload(uint16_t msgId, size_t targetSize)
{
  char headerBuf[64];
  int headerLen = std::snprintf(headerBuf, sizeof(headerBuf), "ID:%u|SIZE:%u|PAT:", (unsigned)msgId, (unsigned)targetSize);

  std::vector<uint8_t> data(targetSize);
  size_t copyLen = std::min((size_t)headerLen, targetSize);
  std::memcpy(data.data(), headerBuf, copyLen);

  for (size_t i = copyLen; i < targetSize; ++i)
  {
    data[i] = static_cast<uint8_t>('A' + ((i - copyLen) % 26));
  }

  return data;
}

static bool verifyPayloadPattern(const std::vector<uint8_t> &payload, unsigned int &outMsgId)
{
  outMsgId = 0;
  if (payload.empty())
    return false;

  std::string txt(payload.begin(), payload.begin() + std::min((size_t)40, payload.size()));
  unsigned int msgId = 0;
  unsigned int expectedSize = 0;
  if (sscanf(txt.c_str(), "ID:%u|SIZE:%u|PAT:", &msgId, &expectedSize) < 2)
  {
    return false;
  }
  outMsgId = msgId;

  if (payload.size() != expectedSize)
  {
    return false;
  }

  char headerBuf[64];
  int headerLen = std::snprintf(headerBuf, sizeof(headerBuf), "ID:%u|SIZE:%u|PAT:", msgId, expectedSize);
  size_t copyLen = std::min((size_t)headerLen, payload.size());

  for (size_t i = copyLen; i < payload.size(); ++i)
  {
    uint8_t expectedByte = static_cast<uint8_t>('A' + ((i - copyLen) % 26));
    if (payload[i] != expectedByte)
    {
      return false;
    }
  }

  return true;
}

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "=== LAMP HARDWARE STRESS & EDGE-CASE SUITE ===");

  gpio_reset_pin(HELTEC_POWER_CTRL);
  gpio_set_direction(HELTEC_POWER_CTRL, GPIO_MODE_OUTPUT);
  gpio_set_level(HELTEC_POWER_CTRL, 0);
  vTaskDelay(pdMS_TO_TICKS(100));

  oledErr = oled.init();
  updateOled("Init", "Starting...");

  hal->init();
  int state = radio.begin(868.0);
  if (state != RADIOLIB_ERR_NONE)
  {
    ESP_LOGE(TAG, "Radio Init Failed: %d", state);
    while (1)
      vTaskDelay(1000);
  }

  radio.setSpreadingFactor(7);
  radio.setBandwidth(500.0);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);
  radio.setOutputPower(10);
  radio.setPreambleLength(8);

  protocol->setYieldCallback([]()
                             { vTaskDelay(1); });
  protocol->setVerbose(false);

  // Reliable transfers need a unicast destination: broadcast frames cannot request a SACK.
#ifdef NODE_MODE_RX
  protocol->setNodeAddress(PAPER_TEST_ADDR_RX);
#else
  protocol->setNodeAddress(PAPER_TEST_ADDR_TX);
#endif

#ifdef NODE_MODE_RX
  ESP_LOGI(TAG, "Running in RECEIVER Mode.");
  updateOled("RX MODE", "Listening...");

#ifdef PAPER_TEST_DROP_RATE
  simulatedDropRate = PAPER_TEST_DROP_RATE;
#endif

  protocol->setDropPacketCallback([](const Packet &packet) -> bool
                                  {
    if (simulatedDropRate == 0)
      return false;
    if (packet.header.flags & FLAG_ACK)
      return false;

    int roll = esp_random() % 100;
    if (roll < simulatedDropRate)
    {
      ESP_LOGW(TAG, "DATA_CSV_RX_DROP,%u,%u", (unsigned)packet.header.messageId, (unsigned)packet.header.chunkIndex);
      return true;
    }
    return false; });

  protocol->setOnReceiveCallback([](const std::vector<uint8_t> &payload, float rssi, float snr)
                                 {
    unsigned int msgId = 0;
    bool valid = verifyPayloadPattern(payload, msgId);

    ESP_LOGI(TAG, "DATA_CSV_RX,%u,%u,%.1f,%.1f,%d,%lu",
             msgId, (unsigned)payload.size(), rssi, snr, valid ? 1 : 0, (unsigned long)hal->millis());

    if (!valid)
    {
      ESP_LOGE(TAG, "CORRUPTION_DETECTED in MsgID %u! Size=%u", msgId, (unsigned)payload.size());
    }

    updateOled("RX Msg " + std::to_string(msgId), valid ? "VALID OK" : "CORRUPT!"); });

  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  radio.startReceive();

  std::string rxCmd = "";
  while (true)
  {
    protocol->update(pdTICKS_TO_MS(xTaskGetTickCount()));

    int c = getchar();
    if (c != EOF)
    {
      rxCmd += (char)c;
      if (rxCmd.length() > 30)
      {
	rxCmd.erase(0, 1);
      }
      if (rxCmd.find("SET_DROP_20") != std::string::npos)
      {
	simulatedDropRate = 20;
	ESP_LOGI(TAG, "RECEIVER DROP RATE SET TO 20%%");
	updateOled("RX MODE", "Drop: 20%");
	rxCmd.clear();
      }
      else if (rxCmd.find("SET_DROP_100") != std::string::npos)
      {
	simulatedDropRate = 100;
	ESP_LOGI(TAG, "RECEIVER DROP RATE SET TO 100%%");
	updateOled("RX MODE", "Drop: 100%");
	rxCmd.clear();
      }
      else if (rxCmd.find("SET_DROP_0") != std::string::npos)
      {
	simulatedDropRate = 0;
	ESP_LOGI(TAG, "RECEIVER DROP RATE SET TO 0%%");
	updateOled("RX MODE", "Drop: 0%");
	rxCmd.clear();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

#else  // NODE_MODE_TX

  ESP_LOGI(TAG, "Running in TRANSMITTER Mode.");
  ESP_LOGI(TAG, "Waiting for 'START_CALIB', 'START_VALID', 'START_ROBUST', or 'START_EDGE' from serial...");
  updateOled("TX MODE", "Wait Command...");

  std::string cmd = "";
  bool isCalibration = false;
  bool isRobustness = false;
  bool isEdgeCases = false;

  while (true)
  {
    int c = getchar();
    if (c != EOF)
    {
      cmd += (char)c;
      if (cmd.length() > 30)
      {
	cmd.erase(0, 1);
      }
      if (cmd.find("START_CALIB") != std::string::npos)
      {
	isCalibration = true;
	break;
      }
      if (cmd.find("START_VALID") != std::string::npos)
      {
	isCalibration = false;
	break;
      }
      if (cmd.find("START_ROBUST") != std::string::npos)
      {
	isRobustness = true;
	break;
      }
      if (cmd.find("START_EDGE") != std::string::npos)
      {
	isEdgeCases = true;
	break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (isCalibration)
  {
    ESP_LOGI(TAG, "START_CALIB received! Calibration run...");
    currentPayloadSizes = {250};
    currentIterations = 10;
  }
  else if (isEdgeCases)
  {
    ESP_LOGI(TAG, "START_EDGE received! Running EDGE-CASE STRESS SUITE (Loss, Timeout Exceeded, Chunk Boundaries)...");
    currentPayloadSizes = {246, 247, 1000, 5000};
    currentIterations = 5;
  }
  else if (isRobustness)
  {
    ESP_LOGI(TAG, "START_ROBUST received! Running edge-case robustness test suite...");
    currentPayloadSizes = {1, 246, 247, 492, 1000, 2000, 5000};
    currentIterations = 10;
  }
  else
  {
    ESP_LOGI(TAG, "START_VALID received! Validation run...");
    currentPayloadSizes = {250, 1000, 5000};
    currentIterations = 30;
  }

  updateOled("TX MODE", "Starting...");
  vTaskDelay(pdMS_TO_TICKS(1000));

  uint16_t msgIdCounter = 1;

  auto runScenario = [&](const char *scenarioName, bool reliable)
  {
    ESP_LOGI(TAG, "--- STARTING SCENARIO: %s ---", scenarioName);

    for (size_t size : currentPayloadSizes)
    {
      for (int i = 0; i < currentIterations; ++i)
      {
	std::vector<uint8_t> txData = generatePatternPayload(msgIdCounter, size);

	uint32_t startMs = hal->millis();

	auto statsBefore = protocol->getStats();
	bool success = protocol->send(PAPER_TEST_ADDR_RX, txData, reliable);
	auto statsAfter = protocol->getStats();

	uint32_t durationMs = hal->millis() - startMs;
	uint32_t chunksTx = statsAfter.chunksTx - statsBefore.chunksTx;
	uint32_t expectedChunks = (size + LORA_MAX_PAYLOAD_SIZE - 1) / LORA_MAX_PAYLOAD_SIZE;
	uint32_t retries = (chunksTx > expectedChunks) ? (chunksTx - expectedChunks) : 0;

	ESP_LOGI(TAG, "DATA_CSV_TX,%s,%u,%u,%d,%d,%lu,%lu,%lu,%lu",
	         scenarioName, (unsigned)msgIdCounter, (unsigned)size, reliable ? 1 : 0, success ? 1 : 0,
	         (unsigned long)durationMs, (unsigned long)chunksTx, (unsigned long)retries, (unsigned long)hal->millis());

	updateOled("TX Msg " + std::to_string(msgIdCounter), success ? "OK" : "FAIL");

	msgIdCounter++;
	vTaskDelay(pdMS_TO_TICKS(400));
      }
    }
  };

  if (isEdgeCases)
  {
    // 1. Lossless Reliable
    runScenario("CASE_EDGE_LOSSLESS_RELIABLE", true);

    // 2. Retry Exhaustion Test (Timeout Failure Verification)
    ESP_LOGI(TAG, "--- STARTING SCENARIO: CASE_EDGE_RETRY_EXHAUSTION ---");
    ESP_LOGI(TAG, "Simulating unreachable receiver... Expecting send() to return false after MAX_RETRIES (5 retries)...");

    // We send a message that requires ACK, but we know receiver won't respond or drop rate is high
    std::vector<uint8_t> dummyData = generatePatternPayload(9999, 1000);
    uint32_t startFailMs = hal->millis();
    bool failOutcome = protocol->send(PAPER_TEST_ADDR_RX, dummyData, true);  // Should return false
    uint32_t failDurationMs = hal->millis() - startFailMs;

    ESP_LOGI(TAG, "DATA_CSV_TX,CASE_EDGE_RETRY_EXHAUSTION,9999,1000,1,%d,%lu,0,5,%lu",
             failOutcome ? 1 : 0, (unsigned long)failDurationMs, (unsigned long)hal->millis());

    if (!failOutcome)
    {
      ESP_LOGI(TAG, "SUCCESS: Max Retries Exhaustion handled gracefully! send() returned false in %lums without hanging.", (unsigned long)failDurationMs);
    }
    else
    {
      ESP_LOGE(TAG, "FAILURE: send() unexpectedly returned true during unreachable receiver test!");
    }
  }
  else
  {
    runScenario("CASE_UNRELIABLE", false);
    runScenario("CASE_RELIABLE", true);
  }

  ESP_LOGI(TAG, "ALL SCENARIOS COMPLETE. IDLE.");
  updateOled("COMPLETE", "Idle");
  while (true)
    vTaskDelay(1000);

#endif
}
#endif
