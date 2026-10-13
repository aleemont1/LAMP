#pragma once

#include <vector>

#include "Packet.hpp"

/**
 * @class PacketSerializer
 * @brief Static utility class for converting between raw data buffers and Packet structures.
 * * This class handles the segmentation of large data arrays into smaller LoRa-compatible
 * packets (splitting) and the serialization of Packet structures into raw byte arrays
 * for transmission.
 */
class PacketSerializer
{
 public:
  /**
   * @brief Serializes a Packet structure into a raw byte buffer.
   * * Copies the header, payload, and CRC into a contiguous memory block
   * ready for hardware transmission.
   * * @param packet The source Packet object.
   * @param buffer The destination buffer. Must be at least MAX_PACKET_SIZE bytes.
   */
  static void serialize(const Packet &packet, uint8_t *buffer);

  /**
   * @brief Splits a raw data buffer into a vector of Packets.
   * * This method calculates the required number of chunks, sets the correct
   * Message ID, chunk indices, and control flags for each packet.
   * * @param data Pointer to the source data.
   * @param length Length of the source data in bytes.
   * @param packetNumberStart The Message ID to assign to these packets (default: 1).
   * @param srcAddr Source node ID (default: ADDRESS_UNASSIGNED).
   * @param dstAddr Destination node ID (default: ADDRESS_BROADCAST).
   * @param maxPayloadSize Payload bytes per chunk (the size negotiated in the handshake), clamped to
   *        [1, LORA_MAX_PAYLOAD_SIZE].
   * @return std::vector<Packet> A list of ready-to-send packets.
   */
  static std::vector<Packet> splitBufferToPackets(const uint8_t *data, size_t length, uint16_t packetNumberStart = 1,
                                                  uint8_t srcAddr = ADDRESS_UNASSIGNED, uint8_t dstAddr = ADDRESS_BROADCAST,
                                                  size_t maxPayloadSize = LORA_MAX_PAYLOAD_SIZE);

  /**
   * @brief Convenience overload for std::vector input.
   * * @param data The source data vector.
   * @param packetNumberStart The Message ID to assign to these packets (default: 1).
   * @param srcAddr Source node ID (default: ADDRESS_UNASSIGNED).
   * @param dstAddr Destination node ID (default: ADDRESS_BROADCAST).
   * @param maxPayloadSize Payload bytes per chunk, clamped to [1, LORA_MAX_PAYLOAD_SIZE].
   * @return std::vector<Packet> A list of ready-to-send packets.
   */
  static std::vector<Packet> splitVectorToPackets(const std::vector<uint8_t> &data, uint16_t packetNumberStart = 1,
                                                  uint8_t srcAddr = ADDRESS_UNASSIGNED, uint8_t dstAddr = ADDRESS_BROADCAST,
                                                  size_t maxPayloadSize = LORA_MAX_PAYLOAD_SIZE);
};