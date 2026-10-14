#include "SackHelper.hpp"

#include <cstring>

std::vector<uint8_t> SackHelper::createFullAckBitmap(uint8_t totalChunks)
{
  size_t bitmapSize = (totalChunks + 7) / 8;
  return std::vector<uint8_t>(bitmapSize, 0xFF);
}

std::vector<uint8_t> SackHelper::getMissingChunkIndices(const std::vector<uint8_t> &bitmap, uint8_t totalChunks)
{
  std::vector<uint8_t> missingIndices;
  for (uint8_t i = 0; i < totalChunks; ++i)
  {
    size_t byteIdx = i / 8;
    size_t bitIdx = i % 8;
    if (byteIdx >= bitmap.size() || !(bitmap[byteIdx] & (1 << bitIdx)))
    {
      missingIndices.push_back(i);
    }
  }
  return missingIndices;
}

Packet SackHelper::createSackPacket(uint16_t messageId, uint8_t totalChunks, const std::vector<uint8_t> &bitmap,
                                    uint8_t srcAddr, uint8_t dstAddr)
{
  static_cast<void>(totalChunks);  // kept in the signature: the SACK always describes a single-frame control message
  Packet ackPacket{};
  ackPacket.header.srcAddr = srcAddr;
  ackPacket.header.dstAddr = dstAddr;
  ackPacket.header.messageId = messageId;
  ackPacket.header.flags = FLAG_ACK;
  ackPacket.header.chunkIndex = 0;
  ackPacket.header.totalChunks = 1;
  ackPacket.header.payloadSize = static_cast<uint8_t>(bitmap.size());

  if (!bitmap.empty())
  {
    std::memcpy(ackPacket.payload.data, bitmap.data(), bitmap.size());
  }

  if (bitmap.size() < LORA_MAX_PAYLOAD_SIZE)
  {
    std::memset(ackPacket.payload.data + bitmap.size(), PAYLOAD_PADDING_BYTE,
                LORA_MAX_PAYLOAD_SIZE - bitmap.size());
  }

  ackPacket.calculateCRC();
  return ackPacket;
}
