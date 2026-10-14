#include "PacketParser.hpp"

#include <cstdio>
#include <cstring>

// Setup logging (compatible with both ESP32 and Native)
static const char *TAG = "PacketParser";
#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <iostream>
#define ESP_LOGE(t, f, ...) printf("ERROR [%s]: " f "\n", t, ##__VA_ARGS__)
#define ESP_LOGW(t, f, ...) printf("WARN  [%s]: " f "\n", t, ##__VA_ARGS__)
#define ESP_LOGI(t, f, ...) printf("INFO  [%s]: " f "\n", t, ##__VA_ARGS__)
#endif

std::optional<Packet> PacketParser::parse(const uint8_t *buffer, size_t length)
{
  // 1. Minimum Size Check
  // A packet must at least have Header + CRC (even with empty payload)
  if (buffer == nullptr || length < HEADER_SIZE + CRC_SIZE)
  {
    ESP_LOGW(TAG, "Drop: Buffer too small (%u bytes)", (unsigned)length);
    return std::nullopt;
  }

  Packet packet{};  // value-initialised: no garbage in the padding

  // 2. Header Extraction
  std::memcpy(&packet.header, buffer, HEADER_SIZE);

  // 3. Size Calculation and Verification
  // Must the received length match what is declared in the header?
  // Calculate how much payload is actually in the buffer.
  size_t actualPayloadSize = length - HEADER_SIZE - CRC_SIZE;

  // If the header says something else, it is a first sign of corruption or logical error
  if (packet.header.payloadSize != actualPayloadSize)
  {
    ESP_LOGW(TAG, "Size Mismatch! BufferLen: %u implies Payload: %u, but Header says: %u",
             (unsigned)length, (unsigned)actualPayloadSize, (unsigned)packet.header.payloadSize);
    return std::nullopt;  // Strict mode: reject malformed frames
  }

  // 4. Payload Extraction
  if (actualPayloadSize > 0)
  {
    if (actualPayloadSize > LORA_MAX_PAYLOAD_SIZE)
    {
      ESP_LOGE(TAG, "Excessive payload size (%u bytes)", (unsigned)actualPayloadSize);
      return std::nullopt;
    }
    // Copy data from buffer (which starts right after the header)
    std::memcpy(packet.payload.data, buffer + HEADER_SIZE, actualPayloadSize);
  }

  // 5. CRC Extraction
  // CRC is located at the END of the received buffer
  uint16_t receivedCrc;
  std::memcpy(&receivedCrc, buffer + length - CRC_SIZE, CRC_SIZE);
  packet.crc = receivedCrc;

  // --- DEBUG LOGGING ---
  // Print what we parsed from the packet BEFORE validating it
  ESP_LOGI(TAG, "RX PARSE: ID=%u Chunk=%u/%u Len=%u CRC=0x%04X",
           (unsigned)packet.header.messageId,
           (unsigned)(packet.header.chunkIndex + 1),  // 1-based index for visual consistency
           (unsigned)packet.header.totalChunks,
           (unsigned)packet.header.payloadSize,
           (unsigned)packet.crc);
  // ---------------------

  // 6. Validation
  auto validationError = PacketValidator::validate(packet);
  if (validationError.has_value())
  {
    ESP_LOGE(TAG, "Validation Failed: %s", validationError.value().details.c_str());

    // Extra debug info for CRC mismatch
    if (validationError.value().type == ValidationError::Type::CRC_MISMATCH)
    {
      Packet temp = packet;
      temp.crc = 0;
      temp.calculateCRC();
      ESP_LOGE(TAG, "Expected CRC (Calc): 0x%04X vs Received: 0x%04X", temp.crc, packet.crc);
    }

    return std::nullopt;
  }

  return packet;
}