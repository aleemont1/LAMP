// Hardware bench for the paper: frame-loss emulation, 4 LAMP modes, line-based serial control.
// Driven by simulation/scripts/bench_hw.py. 869.525 MHz, BW 125 kHz, CR 4/5, 8-symbol preamble.
//
// RX commands : "SF <n>", "DROP <percent>"          (drops data/handshake frames, never SACKs, like the simulator's --loss)
// TX commands : "SF <n>", "DROPACK <percent>" (drops SACK and SYN-ACK frames before the transmitter sees them),
//               "TRIAL <mode 1-4> <chunks>"
// Output lines (CSV, prefixed): RXF (every frame seen by the RX), RXMSG (message completed), TX (one trial result).
#ifdef RUN_BENCH
#include <RadioLib.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "EspHal.hpp"
#include "LoRaProtocol.hpp"
#include "Ssd1306.hpp"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Bench";

EspHal hal_inst(HELTEC_LORA_SCK, HELTEC_LORA_MISO, HELTEC_LORA_MOSI);
EspHal *hal = &hal_inst;
Module radioModule_inst(hal, HELTEC_LORA_NSS, HELTEC_LORA_DIO1, HELTEC_LORA_RST, HELTEC_LORA_BUSY);
SX1262 radio(&radioModule_inst);
LoRaProtocol protocol_inst(&radio, hal, HELTEC_LORA_DIO1);
LoRaProtocol *protocol = &protocol_inst;

static const uint8_t ADDR_TX = 0x01;
static const uint8_t ADDR_RX = 0x02;
static const int MAX_CONNECT_ATTEMPTS = 1 + LoRaMultiPacketConfig::MAX_RETRIES;  // simulator: SYN + R_max resends
static int dropPercent = 0;     // RX: data and handshake frames dropped before the protocol
static int dropAckPercent = 0;  // TX: SACK and SYN-ACK frames dropped before the protocol
static int currentSf = 7;

// ---- OLED status (updated by a low-priority task on core 1; never touches the protocol timing) ----
static Ssd1306 oled;
static bool oledOk = false;
static char note[17] = "";  // free text sent by the host ("NOTE ...")
// RX counters
static volatile uint32_t cFrames = 0, cDropped = 0, cMsgs = 0;
static volatile float lastRssi = 0, lastSnr = 0;
// TX state
static volatile int txMode = 0, txTrial = 0, txDone = 0, txFail = 0;
static volatile bool txRunning = false, txLastOk = false;
static volatile uint32_t txStartMs = 0, txLastDurMs = 0, txLastChunks = 0;

static void oledLine(int row, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void oledLine(int row, const char *fmt, ...)
{
  char buf[24];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  buf[16] = '\0';  // 16 columns
  oled.print(row, 0, buf);
}

static void oledTask(void *)
{
  while (true)
  {
    oled.clear();
#ifdef NODE_MODE_RX
    oledLine(0, "LAMP BENCH  RX");
    oledLine(1, "SF%d BW125 10dBm", currentSf);
    oledLine(2, "drop: %d%%", dropPercent);
    oledLine(3, "frames: %lu", (unsigned long)cFrames);
    oledLine(4, "dropped: %lu", (unsigned long)cDropped);
    oledLine(5, "msgs ok: %lu", (unsigned long)cMsgs);
    oledLine(6, "R%.0f S%.1f", lastRssi, lastSnr);
#else
    oledLine(0, "LAMP BENCH  TX");
    oledLine(1, "SF%d BW125 10dBm", currentSf);
    if (txRunning)
    {
      oledLine(2, "Mode %d  #%d", txMode, txTrial);
      oledLine(3, "RUNNING %lus", (unsigned long)((hal->millis() - txStartMs) / 1000));
    }
    else if (txTrial > 0)
    {
      oledLine(2, "Mode %d  #%d", txMode, txTrial);
      oledLine(3, "%s", txLastOk ? "DONE ok" : "DONE FAIL");
      oledLine(4, "%lums", (unsigned long)txLastDurMs);
      oledLine(5, "chunks tx: %lu", (unsigned long)txLastChunks);
    }
    else
    {
      oledLine(2, "waiting host...");
    }
    oledLine(6, "ok:%d fail:%d", txDone, txFail);
#endif
    oledLine(7, "%s", note);
    oled.update();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static bool readLine(std::string &line)
{
  int c;
  while ((c = getchar()) != EOF)
  {
    if (c == '\n' || c == '\r')
    {
      if (!line.empty())
	return true;
      continue;
    }
    line += static_cast<char>(c);
  }
  return false;
}

static void setSf(int sf)
{
  currentSf = sf;
  radio.setSpreadingFactor(sf);
  printf("OK,SF,%d\n", sf);
}

static std::vector<uint8_t> makePayload(size_t size)
{
  std::vector<uint8_t> data(size);
  for (size_t i = 0; i < size; ++i)
    data[i] = static_cast<uint8_t>('A' + (i % 26));
  return data;
}

extern "C" void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_WARN);  // keep the serial line free for CSV

  gpio_reset_pin(HELTEC_POWER_CTRL);
  gpio_set_direction(HELTEC_POWER_CTRL, GPIO_MODE_OUTPUT);
  gpio_set_level(HELTEC_POWER_CTRL, 0);
  vTaskDelay(pdMS_TO_TICKS(100));

  oledOk = (oled.init() == ESP_OK);
  if (oledOk)
    xTaskCreatePinnedToCore(oledTask, "oled", 4096, nullptr, 1, nullptr, 1);

  hal->init();
  int state = radio.begin(869.525);
  if (state != RADIOLIB_ERR_NONE)
  {
    printf("ERR,radio_init,%d\n", state);
    while (1)
      vTaskDelay(1000);
  }
  radio.setSpreadingFactor(7);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);
  radio.setOutputPower(10);
  radio.setPreambleLength(8);

  protocol->setYieldCallback([]()
                             { vTaskDelay(1); });
  protocol->setVerbose(false);

#ifdef NODE_MODE_RX
  protocol->setNodeAddress(ADDR_RX);

  protocol->setDropPacketCallback([](const Packet &packet) -> bool
                                  {
    bool drop = false;
    if (!(packet.header.flags & FLAG_ACK) && dropPercent > 0)
      drop = static_cast<int>(esp_random() % 100) < dropPercent;
    cFrames = cFrames + 1;
    if (drop)
      cDropped = cDropped + 1;
    lastRssi = radio.getRSSI();
    lastSnr = radio.getSNR();
    printf("RXF,%u,%u,%u,0x%02X,%d,%.1f,%.1f\n", (unsigned)packet.header.messageId, (unsigned)packet.header.chunkIndex,
           (unsigned)packet.header.totalChunks, (unsigned)packet.header.flags, drop ? 1 : 0, radio.getRSSI(), radio.getSNR());
    return drop; });

  protocol->setOnReceiveCallback([](const std::vector<uint8_t> &payload, float rssi, float snr)
                                 { cMsgs = cMsgs + 1;
                                   printf("RXMSG,%u,%.1f,%.1f,%lu\n", (unsigned)payload.size(), rssi, snr, (unsigned long)hal->millis()); });

  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  radio.startReceive();
  printf("READY,RX\n");

  std::string line;
  while (true)
  {
    protocol->update(pdTICKS_TO_MS(xTaskGetTickCount()));

    // Non-blocking line assembly: getchar() returns EOF when no byte is pending.
    int c;
    while ((c = getchar()) != EOF)
    {
      if (c != '\n' && c != '\r')
      {
	line += static_cast<char>(c);
	continue;
      }
      int v = 0;
      if (std::sscanf(line.c_str(), "SF %d", &v) == 1)
      {
	setSf(v);
	radio.startReceive();
      }
      else if (std::sscanf(line.c_str(), "DROP %d", &v) == 1)
      {
	dropPercent = v;
	printf("OK,DROP,%d\n", v);
      }
      else if (line.compare(0, 5, "NOTE ") == 0)
      {
	std::snprintf(note, sizeof(note), "%s", line.c_str() + 5);
      }
      line.clear();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

#else  // NODE_MODE_TX
  protocol->setNodeAddress(ADDR_TX);
  // The transmitter's callback is only consulted for the frames it waits for (SACK, SYN-ACK).
  protocol->setDropPacketCallback([](const Packet &) -> bool
                                  { return dropAckPercent > 0 && static_cast<int>(esp_random() % 100) < dropAckPercent; });
  printf("READY,TX\n");

  std::string line;
  while (true)
  {
    int c = getchar();
    if (c == EOF)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c != '\n' && c != '\r')
    {
      line += static_cast<char>(c);
      continue;
    }

    int a = 0, b = 0;
    if (std::sscanf(line.c_str(), "SF %d", &a) == 1)
    {
      setSf(a);
    }
    else if (line.compare(0, 5, "NOTE ") == 0)
    {
      std::snprintf(note, sizeof(note), "%s", line.c_str() + 5);
    }
    else if (std::sscanf(line.c_str(), "DROPACK %d", &a) == 1)
    {
      dropAckPercent = a;
      printf("OK,DROPACK,%d\n", a);
    }
    else if (std::sscanf(line.c_str(), "TRIAL %d %d", &a, &b) == 2)
    {
      const int mode = a;
      txMode = mode;
      txTrial = txTrial + 1;
      txStartMs = hal->millis();
      txRunning = true;
      const bool stateful = (mode == 2 || mode == 4);
      const bool reliable = (mode == 3 || mode == 4);
      const auto before = protocol->getStats();
      const uint32_t startMs = hal->millis();

      bool ok = true;
      if (stateful)
      {
	ok = false;
	for (int attempt = 0; attempt < MAX_CONNECT_ATTEMPTS && !ok; ++attempt)
	  ok = protocol->connect(ADDR_RX);
      }
      if (ok)
	ok = protocol->send(ADDR_RX, makePayload(static_cast<size_t>(b) * LORA_MAX_PAYLOAD_SIZE), reliable);
      if (stateful)
	protocol->disconnect();

      const uint32_t durMs = hal->millis() - startMs;
      const auto after = protocol->getStats();
      txRunning = false;
      txLastOk = ok;
      txLastDurMs = durMs;
      txLastChunks = after.chunksTx - before.chunksTx;
      if (ok)
	txDone = txDone + 1;
      else
	txFail = txFail + 1;
      printf("TX,%d,%d,%d,%d,%lu,%lu\n", currentSf, mode, b, ok ? 1 : 0, (unsigned long)durMs,
             (unsigned long)(after.chunksTx - before.chunksTx));
    }
    line.clear();
  }
#endif
}
#endif  // RUN_BENCH
