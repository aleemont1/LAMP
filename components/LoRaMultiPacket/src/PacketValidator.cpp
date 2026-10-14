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
  // Only the combinations of the protocol's flag table are defined (data, ACK_REQ, SACK, SYN, SYN-ACK, ACK, NACK).
  // Reserved bits and any other combination are rejected, so a parser never has to guess their meaning.
  const uint8_t f = header.flags;
  const bool defined = f == 0x00 || f == FLAG_ACK_REQ || f == FLAG_ACK || f == FLAG_CONN_REQ ||
                       f == (FLAG_CONN_REQ | FLAG_CONN_ACK) || f == FLAG_CONN_ACK || f == FLAG_CONN_NACK;
  if (!defined)
  {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "Undefined flag combination 0x%02X", (unsigned)f);
    return ValidationError(ValidationError::Type::INVALID_FLAGS, buf);
  }

  // Control frames are single frames: SACK, SYN, SYN-ACK, ACK and NACK all carry totalChunks = 1, chunkIndex = 0.
  const bool control = (f & (FLAG_ACK | FLAG_CONN_REQ | FLAG_CONN_ACK | FLAG_CONN_NACK)) != 0;
  if (control && (header.totalChunks != 1 || header.chunkIndex != 0))
  {
    return ValidationError(ValidationError::Type::INVALID_FLAGS, "Control frame must have totalChunks = 1, chunkIndex = 0");
  }

  // The final ACK of the handshake has no payload; a SACK carries a bitmap of 1..ceil(255 / 8) bytes.
  if (f == FLAG_CONN_ACK && header.payloadSize != 0)
  {
    return ValidationError(ValidationError::Type::INVALID_FLAGS, "Handshake ACK must not carry a payload");
  }
  if (f == FLAG_ACK && (header.payloadSize < 1 || header.payloadSize > 32))
  {
    return ValidationError(ValidationError::Type::INVALID_FLAGS, "SACK bitmap must be 1..32 bytes");
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
