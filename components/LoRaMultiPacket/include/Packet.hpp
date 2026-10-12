#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @brief Maximum raw packet size assumed for transmit buffers (including header and CRC).
 * Many LoRa modules have a FIFO limit (e.g., 256 bytes for SX127x).
 */
constexpr size_t MAX_PACKET_SIZE = 255;

/**
 * @brief Reserved bytes for future use or driver overhead.
 */
constexpr size_t RESERVED_BYTES = 0;

/**
 * @brief Size of the Cyclic Redundancy Check (CRC) suffix in bytes.
 */
constexpr size_t CRC_SIZE = sizeof(uint16_t);

/**
 * @brief The maximum size available for the packet logic after reservations.
 */
constexpr size_t MAX_TX_PACKET_SIZE = MAX_PACKET_SIZE - RESERVED_BYTES;

/**
 * @name Packet Flags
 * @brief Bitmasks for the PacketHeader 'flags' field.
 *
 * Bit layout (uint8_t):
 *   bit 0 (0x01) - reserved
 *   bit 1 (0x02) - reserved
 *   bit 2 (0x04) - FLAG_ACK_REQ
 *   bit 3 (0x08) - FLAG_ACK
 *   bit 4 (0x10) - FLAG_CONN_REQ
 *   bit 5 (0x20) - FLAG_CONN_ACK
 *   bit 6 (0x40) - FLAG_CONN_NACK
 *   bit 7 (0x80) - reserved
 *
 * Start/End of message are derived from chunkIndex and totalChunks,
 * which are already present in the header — no redundant flags needed.
 * @{
 */
constexpr uint8_t FLAG_ACK_REQ = 0x04;    ///< Acknowledgement Requested: receiver must send a SACK.
constexpr uint8_t FLAG_ACK = 0x08;        ///< Selective Acknowledgment (SACK) feedback packet.
constexpr uint8_t FLAG_CONN_REQ = 0x10;   ///< Connection Request: negotiate session parameters before transfer.
constexpr uint8_t FLAG_CONN_ACK = 0x20;   ///< Connection Acknowledged: session accepted, parameters confirmed.
constexpr uint8_t FLAG_CONN_NACK = 0x40;  ///< Connection Rejected: session refused (e.g. out of memory).

/** @} */

/**
 * @name Node Address Constants
 * @brief Default node address identifiers.
 * @{
 */
constexpr uint8_t ADDRESS_UNASSIGNED = 0x00;  ///< Unassigned / P2P Unaddressed Mode
constexpr uint8_t ADDRESS_BROADCAST = 0xFF;   ///< Broadcast address (processed by all receiving nodes)

/**
 * @brief Maps a random word to a valid message ID (0 is reserved).
 *
 * Used to start the message ID counter at a random value on each boot, so that a rebooted sender does not reuse
 * the IDs the receiver still remembers.
 */
inline uint16_t messageIdFromRandom(uint32_t random)
{
  uint16_t id = static_cast<uint16_t>(random & 0xFFFF);
  return (id == 0) ? 1 : id;
}
/** @} */

#pragma pack(push, 1)  // Ensure no compiler padding is inserted between fields

/**
 * @brief Header structure containing metadata for segmentation and reassembly.
 * Total size: 9 bytes.
 */
struct PacketHeader
{
  /**
   * @brief Source node identifier.
   */
  uint8_t srcAddr = ADDRESS_UNASSIGNED;

  /**
   * @brief Destination node identifier (0xFF for broadcast).
   */
  uint8_t dstAddr = ADDRESS_BROADCAST;

  /**
   * @brief Unique identifier for a complete message.
   * All chunks belonging to the same large message must share this ID.
   */
  uint16_t messageId = 1;

  /**
   * @brief Total number of chunks this message is split into.
   */
  uint8_t totalChunks = 0;

  /**
   * @brief The sequence index of this specific chunk (0-based).
   * Used to reorder packets if they arrive out of order.
   */
  uint8_t chunkIndex = 0;

  /**
   * @brief Number of valid data bytes in the payload of this packet.
   * Must be <= LORA_MAX_PAYLOAD_SIZE.
   */
  uint8_t payloadSize = 0;

  /**
   * @brief Bitmask of packet control flags.
   * See FLAG_* constants. SOM/EOM are not encoded here: use chunkIndex and totalChunks instead.
   */
  uint8_t flags = 0;

  /**
   * @brief Protocol version for compatibility checks.
   */
  uint8_t protocolVersion = 2;
};

/**
 * @brief Size of the packet header in bytes.
 */
constexpr size_t HEADER_SIZE = sizeof(PacketHeader);

/**
 * @brief Maximum bytes available for actual data payload per packet.
 * Calculated as: Total Available - Header - CRC.
 */
constexpr size_t LORA_MAX_PAYLOAD_SIZE = MAX_TX_PACKET_SIZE - HEADER_SIZE - CRC_SIZE;

/**
 * @brief Padding byte value used to fill unused space in the final packet's payload.
 * When the last chunk contains fewer bytes than LORA_MAX_PAYLOAD_SIZE, remaining slots
 * are filled with this value to maintain fixed-size serialization.
 */
constexpr uint8_t PAYLOAD_PADDING_BYTE = 0xFF;

/**
 * @brief Fixed-size container for payload data.
 */
struct PacketPayload
{
  uint8_t data[LORA_MAX_PAYLOAD_SIZE]{0};
};

/**
 * @brief The complete Over-The-Air (OTA) packet structure.
 * This structure maps directly to the byte array sent to the LoRa modem.
 */
struct Packet
{
  PacketHeader header{};    ///< Metadata for transport.
  PacketPayload payload{};  ///< Application data segment.
  uint16_t crc{0};          ///< Error detection checksum.

  /**
   * @brief Computes the CRC-16 of the packet without modifying state.
   */
  uint16_t computeCRC() const;

  /**
   * @brief Calculates the CRC-16 of the packet and updates the 'crc' field.
   *
   * **CRC Scope:** Covers the full header + only valid payload bytes (respects payloadSize).
   * Padding bytes in the payload buffer are explicitly excluded from CRC calculation.
   *
   * Rationale: This design decouples integrity checking from physical layout and padding
   * strategy. A bit flip in unused padding does not cause a false CRC failure. The CRC protects
   * the semantic message content, not transport artifacts used to fit hardware constraints.
   *
   * Algorithm: Modbus CRC-16 (polynomial 0xA001, initial value 0xFFFF).
   *
   * Formula: CRC(header || payload[0..payloadSize-1])
   */
  void calculateCRC();

  /**
   * @brief Prints a human-readable summary of the packet to the log output.
   * Useful for debugging transmission logic.
   */
  void printPacket() const;
};
#pragma pack(pop)