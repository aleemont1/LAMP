#include "PacketValidator.hpp"

#include <cstdio>

std::optional<ValidationError> PacketValidator::validate(const Packet &packet)
{
  // Validate header
  auto headerErr = validateHeader(packet.header);
  if (headerErr.has_value())
    return headerErr;

  // Validate flags
  auto flagErr = validateFlags(packet.header);
  if (flagErr.has_value())
    return flagErr;

  // Validate CRC
  auto crcErr = validateCRC(packet, packet.crc);
  if (crcErr.has_value())
    return crcErr;

  return std::nullopt;
}

std::optional<ValidationError> PacketValidator::validateHeader(
    const PacketHeader &header)
{
  // Check protocol version
  if (header.protocolVersion != SUPPORTED_PROTOCOL_VERSION)
  {
    return ValidationError(
        ValidationError::Type::INVALID_PROTOCOL_VERSION,
        "Protocol version " + std::to_string(header.protocolVersion) +
            " not supported (expected " +
            std::to_string(SUPPORTED_PROTOCOL_VERSION) + ")");
  }

  // Check message ID (0 is reserved)
  if (header.messageId == 0)
  {
    return ValidationError(ValidationError::Type::INVALID_MESSAGE_ID,
                           "Message ID cannot be 0 (reserved value)");
  }

  // Check totalChunks
  if (header.totalChunks == 0)
  {
    return ValidationError(ValidationError::Type::INVALID_TOTAL_CHUNKS,
                           "totalChunks must be >= 1");
  }

  // Check chunkIndex within bounds
  if (header.chunkIndex >= header.totalChunks)
  {
    return ValidationError(
        ValidationError::Type::INVALID_CHUNK_INDEX,
        "chunkIndex (" + std::to_string(header.chunkIndex) +
            ") >= totalChunks (" + std::to_string(header.totalChunks) + ")");
  }

  // Check payloadSize within bounds
  if (header.payloadSize > LORA_MAX_PAYLOAD_SIZE)
  {
    return ValidationError(
        ValidationError::Type::INVALID_PAYLOAD_SIZE,
        "payloadSize (" + std::to_string(header.payloadSize) +
            ") > LORA_MAX_PAYLOAD_SIZE (" +
            std::to_string(LORA_MAX_PAYLOAD_SIZE) + ")");
  }

  // Logical check: a non-final chunk carries data. Its size is the payload size negotiated in the handshake
  // (at most LORA_MAX_PAYLOAD_SIZE), which a stateless validator cannot know.
  bool isLastChunk = (header.chunkIndex == header.totalChunks - 1);
  if (!isLastChunk && header.payloadSize == 0)
  {
    return ValidationError(
        ValidationError::Type::INVALID_PAYLOAD_SIZE,
        "Non-final chunk must carry payload, got 0 bytes");
  }

  return std::nullopt;
}

std::optional<ValidationError> PacketValidator::validateFlags(
    const PacketHeader &header)
{
  // CONN_ACK and CONN_NACK are mutually exclusive
  bool hasConnAck = (header.flags & FLAG_CONN_ACK) != 0;
  bool hasConnNack = (header.flags & FLAG_CONN_NACK) != 0;
  if (hasConnAck && hasConnNack)
  {
    return ValidationError(ValidationError::Type::INVALID_FLAGS,
                           "CONN_ACK and CONN_NACK cannot both be set");
  }

  return std::nullopt;
}

std::optional<ValidationError> PacketValidator::validateCRC(
    const Packet &packet, uint16_t receivedCrc)
{
  uint16_t calculatedCrc = packet.computeCRC();

  // Compare calculated CRC with received CRC
  if (calculatedCrc != receivedCrc)
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "CRC mismatch: expected 0x%04X, received 0x%04X", calculatedCrc, receivedCrc);
    return ValidationError(
        ValidationError::Type::CRC_MISMATCH,
        std::string(buf));
  }

  return std::nullopt;
}
