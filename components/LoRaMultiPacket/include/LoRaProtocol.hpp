#pragma once

#include <RadioLib.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "Connection.hpp"
#include "Packet.hpp"
#include "PacketParser.hpp"
#include "PacketReassembler.hpp"
#include "PacketSerializer.hpp"
#include "ProtocolConfig.hpp"
#include "SackHelper.hpp"

/**
 * @class LoRaProtocol
 * @brief Manages fragmentation, transmission, reassembly, and connection lifecycle of LoRa packets.
 */
class LoRaProtocol
{
 public:
  using OnReceiveCallback = std::function<void(const std::vector<uint8_t> &data, float rssi, float snr)>;
  using OnConnectionRequestCallback = std::function<bool(SynMetadata &syn)>;
  using YieldCallback = std::function<void()>;
  using DropPacketCallback = std::function<bool(const Packet &packet)>;
  using FreeHeapProvider = std::function<size_t()>;

  /**
   * @brief Constructor accepting the specific SX1262 driver, HAL pointer, and physical IRQ pin.
   * @param randomSource Source of the random word that seeds the message ID counter (default: the SX1262 random
   *        generator mixed with the ESP32 one). The counter is seeded on first use, not here, because the radio is
   *        not initialised yet when the object is constructed. It starts at a random value on each boot so that a
   *        restarted node does not reuse the IDs the receiver still remembers.
   */
  explicit LoRaProtocol(SX1262 *radio, RadioLibHal *hal, uint32_t irqPin,
                        std::function<uint32_t()> randomSource = nullptr);

  /**
   * @brief Sets the local node address (0x01..0xFE).
   */
  void setNodeAddress(uint8_t address) { nodeAddress_ = address; }

  /**
   * @brief Returns the local node address.
   */
  uint8_t getNodeAddress() const { return nodeAddress_; }

  /**
   * @brief Initiates a 3-way handshake (SYN, SYN-ACK, ACK) to establish a connection session with a target node. Blocking.
   * @param targetAddress Address of the destination node.
   * @param timeoutMs Maximum time to wait for handshake completion.
   * @return true if connection established successfully, false otherwise.
   */
  bool connect(uint8_t targetAddress, uint32_t timeoutMs = LoRaMultiPacketConfig::DEFAULT_SYN_TIMEOUT_MS);

  /**
   * @brief Initiates a 3-way handshake with the broadcast target. Overload kept for backwards compatibility;
   * it always fails, because a handshake needs a unicast target.
   */
  bool connect(uint32_t timeoutMs = LoRaMultiPacketConfig::DEFAULT_SYN_TIMEOUT_MS)
  {
    return connect(ADDRESS_BROADCAST, timeoutMs);
  }

  /**
   * @brief Teardown active connection session.
   */
  void disconnect();

  /**
   * @brief Returns true if a connection session is currently ESTABLISHED.
   */
  bool isConnected() const { return connection_.state == ConnectionState::ESTABLISHED; }

  /**
   * @brief Returns current connection state.
   */
  ConnectionState getConnectionState() const { return connection_.state; }

  /**
   * @brief Sends a payload to a specific target node. Blocking.
   * @note reliable = true requires a unicast target; with ADDRESS_BROADCAST it returns false without transmitting.
   */
  bool send(uint8_t targetAddress, const std::vector<uint8_t> &data, bool reliable = false);

  /**
   * @brief Sends a payload to the default broadcast target. Overload for backwards compatibility.
   * @note With reliable = true this fails: a broadcast cannot be acknowledged. Use send(targetAddress, ...).
   */
  bool send(const std::vector<uint8_t> &data, bool reliable = false)
  {
    return send(ADDRESS_BROADCAST, data, reliable);
  }

  /**
   * @brief Main loop update. Handles RX polling, reassembly timeouts, and connection maintenance.
   */
  void update(uint32_t currentTimestampMs);

  void setOnReceiveCallback(OnReceiveCallback callback);
  void setOnConnectionRequestCallback(OnConnectionRequestCallback callback);
  void setYieldCallback(YieldCallback callback);
  void setDropPacketCallback(DropPacketCallback callback);

  /**
   * @brief Overrides the source of the free-heap figure used for receiver admission (default: esp_get_free_heap_size()).
   */
  void setFreeHeapProvider(FreeHeapProvider provider);

  void setVerbose(bool enable);

  struct ProtocolStats
  {
    uint32_t chunksTx = 0;
    uint32_t chunksRx = 0;
    uint32_t packetsRx = 0;
    uint32_t packetsFailed = 0;
    uint32_t packetsDroppedAddress = 0;
    uint32_t packetsTx = 0;
    uint32_t packetsTxFailed = 0;
    uint32_t synSent = 0;
    uint32_t synRcvd = 0;
    uint32_t connEstablished = 0;
    uint32_t connNacked = 0;
  };

  const ProtocolStats &getStats() const { return stats_; }
  void resetStats() { stats_ = ProtocolStats(); }

 private:
  SX1262 *radio_;        ///< Pointer to SX1262 driver
  RadioLibHal *hal_;     ///< Pointer to hardware HAL
  uint32_t irqPin_;      ///< Hardware interrupt pin (e.g. DIO1)
  uint8_t nodeAddress_;  ///< Local node address (0x00=Unassigned, 0x01..0xFE=Node, 0xFF=Broadcast)
  PacketReassembler reassembler_;
  ConnectionSession connection_;
  OnReceiveCallback onReceive_;
  OnConnectionRequestCallback onConnRequest_;
  YieldCallback yieldCallback_;
  DropPacketCallback dropPacketCallback_;
  FreeHeapProvider freeHeapProvider_;
  ProtocolStats stats_;
  bool verbose_;
  std::function<uint32_t()> randomSource_;  ///< optional override of the message ID seed (tests)
  uint16_t nextMessageId_;
  bool messageIdSeeded_;
  uint8_t phyBuffer_[LoRaMultiPacketConfig::PHY_BUFFER_SIZE];  ///< Shared buffer for hardware I/O

  // Returns the next message ID (never 0), seeding the counter with a random value on first use.
  uint16_t allocateMessageId();

  // Internal helper methods for 3-Way Handshake
  void sendSyn(uint8_t targetAddr, uint16_t msgId, const SynMetadata &syn);
  void sendSynAck(uint8_t targetAddr, uint16_t msgId, const SynAckMetadata &synAck);
  void sendConnNack(uint8_t targetAddr, uint16_t msgId, ConnNackReason reason);
  bool waitForSynAck(uint8_t targetAddr, uint16_t msgId, uint32_t timeoutMs, SynAckMetadata &synAckOut);
  bool waitForConnAck(uint16_t msgId, uint32_t timeoutMs);

  // Internal helper methods for transmission
  bool sendUnreliable(const std::vector<Packet> &packets);
  bool sendReliable(const std::vector<Packet> &packets, size_t chunkPayloadSize);
  bool waitForSack(uint8_t targetAddr, uint16_t msgId, uint32_t timeoutMs, std::vector<uint8_t> &sackBitmapOut);
  bool retransmitMissingChunks(const std::vector<Packet> &packets, const std::vector<uint8_t> &missingIndices);
  uint32_t calculateAckTimeoutMs(size_t totalChunks, size_t chunkPayloadSize = LORA_MAX_PAYLOAD_SIZE) const;
  int transmitPacket(const Packet &packet, const char *logPrefix);

  // Internal helper methods for reception
  void sendSACK(uint8_t targetAddr, uint16_t messageId, uint8_t totalChunks, bool allReceived);
  std::optional<Packet> tryReceivePacket();
  void handleIncomingPacket(const Packet &packet, uint32_t currentTimestampMs);
};