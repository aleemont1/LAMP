#include "PacketSerializer.hpp"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <vector>

void PacketSerializer::serialize(const Packet &packet, uint8_t *buffer)
{
  // Copy header
  std::memcpy(buffer, &packet.header, HEADER_SIZE);
  // Copy payload (only the actual valid bytes)
  if (packet.header.payloadSize > 0)
  {
    std::memcpy(buffer + HEADER_SIZE, packet.payload.data, packet.header.payloadSize);
  }
  // Copy CRC (2 bytes) immediately after the payload
  std::memcpy(buffer + HEADER_SIZE + packet.header.payloadSize, &packet.crc,
              CRC_SIZE);
}

std::vector<Packet> PacketSerializer::splitBufferToPackets(const uint8_t *data, size_t length, uint16_t packetNumberStart,
                                                           uint8_t srcAddr, uint8_t dstAddr, size_t maxPayloadSize)
{
  std::vector<Packet> result;
  if (data == nullptr || length == 0)
    return result;

  maxPayloadSize = std::min(std::max<size_t>(maxPayloadSize, 1), LORA_MAX_PAYLOAD_SIZE);

  size_t offset = 0;
  uint16_t messageId = packetNumberStart;
  size_t numChunks = (length + maxPayloadSize - 1) / maxPayloadSize;
  if (numChunks > 255)
  {
    return result;  // Payload exceeds maximum segmentable protocol size
  }
  uint8_t totalChunks = static_cast<uint8_t>(numChunks);

  uint8_t chunkIndex = 0;
  while (offset < length)
  {
    Packet packet{};
    packet.header.srcAddr = srcAddr;
    packet.header.dstAddr = dstAddr;
    packet.header.messageId = messageId;
    packet.header.totalChunks = totalChunks;
    packet.header.chunkIndex = chunkIndex;

    size_t remaining = length - offset;
    uint8_t payloadSize =
        static_cast<uint8_t>(std::min(remaining, maxPayloadSize));
    packet.header.payloadSize = payloadSize;

    std::memcpy(packet.payload.data, data + offset, payloadSize);

    // Fill remaining bytes with padding (all 1s) if payload is not full
    if (payloadSize < LORA_MAX_PAYLOAD_SIZE)
    {
      std::memset(packet.payload.data + payloadSize, PAYLOAD_PADDING_BYTE,
                  LORA_MAX_PAYLOAD_SIZE - payloadSize);
    }

    // flags field: SOM/EOM are not encoded (redundant with chunkIndex/totalChunks).
    // Control flags (ACK_REQ, CONN_REQ, etc.) are set by the caller after split.
    packet.header.flags = 0;

    packet.calculateCRC();
    result.push_back(packet);

    offset += payloadSize;
    chunkIndex++;
  }

  return result;
}

std::vector<Packet> PacketSerializer::splitVectorToPackets(const std::vector<uint8_t> &data, uint16_t packetNumberStart,
                                                           uint8_t srcAddr, uint8_t dstAddr, size_t maxPayloadSize)
{
  return splitBufferToPackets(data.empty() ? nullptr : data.data(), data.size(), packetNumberStart, srcAddr, dstAddr,
                              maxPayloadSize);
}