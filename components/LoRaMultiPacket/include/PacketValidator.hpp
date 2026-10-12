#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "Packet.hpp"

/**
 * @struct ValidationError
 * @brief Details about packet validation failure.
 */
struct ValidationError
{
  enum class Type
  {
    BUFFER_TOO_SMALL,          ///< Provided buffer is smaller than minimum packet size
    INVALID_PROTOCOL_VERSION,  ///< Protocol version not supported
    INVALID_TOTAL_CHUNKS,      ///< totalChunks == 0 or exceeds MAX (255)
    INVALID_CHUNK_INDEX,       ///< chunkIndex >= totalChunks
    INVALID_PAYLOAD_SIZE,      ///< payloadSize > LORA_MAX_PAYLOAD_SIZE or (not last chunk && payloadSize != LORA_MAX_PAYLOAD_SIZE)
    INVALID_MESSAGE_ID,        ///< messageId == 0 (reserved)
    CRC_MISMATCH,              ///< CRC validation failed
    INVALID_FLAGS,             ///< Illegal flag combination detected
  };

  Type type;
  std::string details;

  explicit ValidationError(Type t, const std::string &msg = "") : type(t), details(msg) {}
};

/**
 * @class PacketValidator
 * @brief Validates packet integrity without performing deserialization.
 *
 * Performs multi-stage validation:
 *   1. Header field sanity checks
 *   2. CRC verification (excludes padding as per protocol design)
 *   3. Start/End-of-Message flag consistency
 *
 * If all checks pass, the packet is guaranteed to be safe.
 * On failure, the packet should be discarded.
 *
 * **Note:** This class performs validation only. The PacketDeserializer
 * class calls these validation APIs and performs the actual deserialization.
 */
class PacketValidator
{
 public:
  /**
   * @brief Validates a deserialized packet.
   *
   * Performs comprehensive integrity checks:
   *   - Header fields are within valid ranges
   *   - Message ID is non-zero
   *   - CRC-16 matches calculated value (covers header + valid payload only)
   *   - No illegal flag combinations
   *
   * @param packet The packet to validate
   * @return std::nullopt if valid, ValidationError details if invalid
   */
  static std::optional<ValidationError> validate(const Packet &packet);

  /**
   * @brief True for a broadcast frame that requests an acknowledgment or carries connection control flags.
   *
   * Such frames are meaningless (there is no single peer to answer) and would make every node in range reply at
   * once, so receivers drop them.
   */
  static bool isBroadcastWithReliabilityFlags(const PacketHeader &header)
  {
    constexpr uint8_t kFlags = FLAG_ACK_REQ | FLAG_CONN_REQ | FLAG_CONN_ACK | FLAG_CONN_NACK;
    return header.dstAddr == ADDRESS_BROADCAST && (header.flags & kFlags) != 0;
  }

  /**
   * @brief True if a message to dstAddr may be sent with acknowledgments (i.e. dstAddr is not broadcast).
   */
  static bool canSendReliablyTo(uint8_t dstAddr) { return dstAddr != ADDRESS_BROADCAST; }

 private:
  static constexpr size_t MIN_PACKET_SIZE =
      HEADER_SIZE + sizeof(PacketPayload) + CRC_SIZE;
  static constexpr uint8_t SUPPORTED_PROTOCOL_VERSION = 2;

  /**
   * @brief Validates header fields for sanity.
   */
  static std::optional<ValidationError> validateHeader(
      const PacketHeader &header);

  /**
   * @brief Validates CRC against calculated value.
   *
   * CRC calculation respects the protocol design: covers header + valid payload,
   * explicitly excludes padding bytes.
   */
  static std::optional<ValidationError> validateCRC(const Packet &packet,
                                                    uint16_t receivedCrc);

  /**
   * @brief Validates that no illegal flag combinations are set.
   */
  static std::optional<ValidationError> validateFlags(
      const PacketHeader &header);
};
