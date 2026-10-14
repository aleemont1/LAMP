#include "LoRaProtocol.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <utility>

#include "PacketValidator.hpp"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
static const char *TAG = "LoRaProto";
#else
#define ESP_LOGI(tag, format, ...) printf("INFO  [%s]: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("WARN  [%s]: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) printf("ERROR [%s]: " format "\n", tag, ##__VA_ARGS__)
static const char *TAG = "LoRaProto";
#endif

static size_t defaultFreeHeap()
{
#ifdef ESP_PLATFORM
  return esp_get_free_heap_size();
#else
  return SIZE_MAX;
#endif
}

#ifdef LORA_PROTOCOL_HAS_RADIOLIB
LoRaProtocol::LoRaProtocol(SX1262 *radio, RadioLibHal *hal, uint32_t irqPin, std::function<uint32_t()> randomSource)
    : LoRaProtocol(static_cast<IRadio *>(nullptr), static_cast<IPlatform *>(nullptr), irqPin, std::move(randomSource))
{
  ownedRadio_ = std::make_unique<Sx1262Radio>(radio);
  ownedPlatform_ = std::make_unique<RadioLibPlatform>(hal);
  radio_ = ownedRadio_.get();
  hal_ = ownedPlatform_.get();
}
#endif

LoRaProtocol::LoRaProtocol(IRadio *radio, IPlatform *platform, uint32_t irqPin, std::function<uint32_t()> randomSource)
    : radio_(radio),
      hal_(platform),
      irqPin_(irqPin),
      nodeAddress_(ADDRESS_UNASSIGNED),
      onConnRequest_(nullptr),
      dropPacketCallback_(nullptr),
      freeHeapProvider_(defaultFreeHeap),
      verbose_(false),
      randomSource_(std::move(randomSource)),
      nextMessageId_(1),
      messageIdSeeded_(false),
      phyBuffer_{0}
{
  connection_.state = ConnectionState::CLOSED;
}

uint16_t LoRaProtocol::allocateMessageId()
{
  if (!messageIdSeeded_)
  {
    uint32_t seed;
    if (randomSource_)
    {
      seed = randomSource_();
    }
    else
    {
#ifdef ESP_PLATFORM
      // SX1262 noise source (two bytes; it leaves the radio in standby, the next TX/RX restarts it) mixed with the
      // ESP32 generator, which is only partly random without WiFi/BT.
      uint32_t radioWord = radio_->randomByte();
      radioWord = (radioWord << 8) | radio_->randomByte();
      seed = (radioWord << 16) ^ radioWord ^ esp_random();
#else
      seed = std::random_device{}();
#endif
    }
    nextMessageId_ = messageIdFromRandom(seed);
    messageIdSeeded_ = true;
  }

  uint16_t id = nextMessageId_++;
  if (nextMessageId_ == 0)
  {
    nextMessageId_ = 1;
  }
  return id;
}

bool LoRaProtocol::connect(uint8_t targetAddress, uint32_t timeoutMs)
{
  if (!PacketValidator::canSendReliablyTo(targetAddress))
  {
    ESP_LOGE(TAG, "connect() needs a unicast target: a broadcast SYN would be dropped by every receiver");
    return false;
  }

  // One connection per node. An expired one is closed first; an established one to the same peer is reused; any other
  // state belongs to a different peer or to a handshake in progress and must not be overwritten.
  if (connection_.state != ConnectionState::CLOSED && connection_.inactivityExpired(hal_->millis()))
  {
    connection_.state = ConnectionState::CLOSED;
  }
  if (connection_.state == ConnectionState::ESTABLISHED && connection_.peerAddr == targetAddress)
  {
    return true;
  }
  if (connection_.state != ConnectionState::CLOSED)
  {
    ESP_LOGW(TAG, "connect() refused: a connection with 0x%02X is active (one connection per node)",
             (unsigned)connection_.peerAddr);
    return false;
  }

  uint16_t msgId = allocateMessageId();

  connection_.state = ConnectionState::SYN_SENT;
  connection_.sessionMsgId = msgId;
  connection_.peerAddr = targetAddress;
  stats_.synSent++;

  SynMetadata synReq{};
  synReq.requestedPayloadSize = LORA_MAX_PAYLOAD_SIZE;
  synReq.windowSize = 1;
  synReq.timeoutMs = static_cast<uint16_t>(LoRaMultiPacketConfig::DEFAULT_CONN_INACTIVITY_TIMEOUT_MS);

  ESP_LOGI(TAG, "Initiating 3-Way Handshake (MsgID %u, SYN sent to 0x%02X)...", msgId, (unsigned)targetAddress);
  if (!sendSyn(targetAddress, msgId, synReq))
  {
    ESP_LOGE(TAG, "3-Way Handshake Failed (SYN could not be transmitted)");
    connection_.state = ConnectionState::CLOSED;
    return false;
  }

  SynAckMetadata synAckResp{};
  const SynWait wait = waitForSynAck(targetAddress, msgId, timeoutMs, synAckResp);
  if (wait == SynWait::FAILED)
  {
    ESP_LOGE(TAG, "3-Way Handshake Failed (SYN-ACK Timeout/NACK for MsgID %u)", msgId);
    connection_.state = ConnectionState::CLOSED;
    return false;
  }
  if (wait == SynWait::ESTABLISHED_AS_RESPONDER)
  {
    ESP_LOGI(TAG, "Simultaneous open resolved: connection with 0x%02X established as responder.", (unsigned)targetAddress);
    return true;
  }

  // Send final ACK of the 3-Way Handshake
  Packet ackPacket{};
  ackPacket.header.srcAddr = nodeAddress_;
  ackPacket.header.dstAddr = targetAddress;
  ackPacket.header.messageId = msgId;
  ackPacket.header.totalChunks = 1;
  ackPacket.header.chunkIndex = 0;
  ackPacket.header.payloadSize = 0;
  ackPacket.header.flags = FLAG_CONN_ACK;
  ackPacket.header.protocolVersion = LoRaMultiPacketConfig::PROTOCOL_VERSION;
  ackPacket.calculateCRC();

  if (transmitPacket(ackPacket, "CONN ACK PACKET") != RadioStatus::OK)
  {
    ESP_LOGE(TAG, "3-Way Handshake Failed (final ACK could not be transmitted)");
    connection_.state = ConnectionState::CLOSED;
    return false;
  }

  connection_.state = ConnectionState::ESTABLISHED;
  connection_.negotiatedPayloadSize = synAckResp.acceptedPayloadSize;
  connection_.windowSize = synAckResp.windowSize;
  connection_.timeoutMs = synReq.timeoutMs;
  connection_.lastActivityMs = hal_->millis();
  stats_.connEstablished++;

  ESP_LOGI(TAG, "3-Way Handshake SUCCESS! Connection ESTABLISHED with Peer (MsgID %u).", msgId);
  return true;
}

void LoRaProtocol::disconnect()
{
  if (connection_.state != ConnectionState::CLOSED)
  {
    ESP_LOGI(TAG, "Closing connection session (MsgID %u)...", connection_.sessionMsgId);
    connection_.state = ConnectionState::CLOSED;
  }
}

bool LoRaProtocol::sendSyn(uint8_t targetAddr, uint16_t msgId, const SynMetadata &syn)
{
  Packet p{};
  p.header.srcAddr = nodeAddress_;
  p.header.dstAddr = targetAddr;
  p.header.messageId = msgId;
  p.header.totalChunks = 1;
  p.header.chunkIndex = 0;
  p.header.payloadSize = sizeof(SynMetadata);
  p.header.flags = FLAG_CONN_REQ;
  p.header.protocolVersion = LoRaMultiPacketConfig::PROTOCOL_VERSION;
  std::memcpy(p.payload.data, &syn, sizeof(SynMetadata));
  p.calculateCRC();

  return transmitPacket(p, "SYN PACKET") == RadioStatus::OK;
}

bool LoRaProtocol::sendSynAck(uint8_t targetAddr, uint16_t msgId, const SynAckMetadata &synAck)
{
  Packet p{};
  p.header.srcAddr = nodeAddress_;
  p.header.dstAddr = targetAddr;
  p.header.messageId = msgId;
  p.header.totalChunks = 1;
  p.header.chunkIndex = 0;
  p.header.payloadSize = sizeof(SynAckMetadata);
  p.header.flags = FLAG_CONN_REQ | FLAG_CONN_ACK;
  p.header.protocolVersion = LoRaMultiPacketConfig::PROTOCOL_VERSION;
  std::memcpy(p.payload.data, &synAck, sizeof(SynAckMetadata));
  p.calculateCRC();

  hal_->delay(LoRaMultiPacketConfig::SACK_PREAMBLE_GUARD_DELAY_MS);
  return transmitPacket(p, "SYN-ACK PACKET") == RadioStatus::OK;
}

bool LoRaProtocol::sendConnNack(uint8_t targetAddr, uint16_t msgId, ConnNackReason reason)
{
  Packet p{};
  p.header.srcAddr = nodeAddress_;
  p.header.dstAddr = targetAddr;
  p.header.messageId = msgId;
  p.header.totalChunks = 1;
  p.header.chunkIndex = 0;
  p.header.payloadSize = 1;
  p.header.flags = FLAG_CONN_NACK;
  p.header.protocolVersion = LoRaMultiPacketConfig::PROTOCOL_VERSION;
  p.payload.data[0] = static_cast<uint8_t>(reason);
  p.calculateCRC();

  hal_->delay(LoRaMultiPacketConfig::SACK_PREAMBLE_GUARD_DELAY_MS);
  return transmitPacket(p, "CONN-NACK PACKET") == RadioStatus::OK;
}

static bool addressedTo(uint8_t self, const PacketHeader &h)
{
  return self == ADDRESS_UNASSIGNED || h.dstAddr == self;
}

LoRaProtocol::SynWait LoRaProtocol::waitForSynAck(uint8_t targetAddr, uint16_t msgId, uint32_t timeoutMs,
                                                  SynAckMetadata &synAckOut)
{
  uint32_t startMs = hal_->millis();
  bool yielded = false;  // simultaneous open: we accepted the peer's SYN and wait for its final ACK instead

  while (hal_->millis() - startMs < timeoutMs)
  {
    auto pktOpt = tryReceivePacket();
    if (pktOpt.has_value())
    {
      const Packet &pkt = pktOpt.value();
      const bool pureSyn = pkt.header.flags == FLAG_CONN_REQ;

      if (!addressedTo(nodeAddress_, pkt.header))
      {
	// not for this node: a SYN-ACK or NACK meant for somebody else must not complete our handshake
      }
      else if (pureSyn && pkt.header.srcAddr != targetAddr)
      {
	handleSyn(pkt, hal_->millis());  // a third node: answered BUSY while our handshake is open
      }
      else if (pkt.header.srcAddr != targetAddr)
      {
	// other frames of other nodes are not for this wait
      }
      else if (yielded)
      {
	if (pkt.header.flags == FLAG_CONN_ACK && pkt.header.messageId == connection_.sessionMsgId)
	{
	  connection_.state = ConnectionState::ESTABLISHED;
	  connection_.lastActivityMs = hal_->millis();
	  stats_.connEstablished++;
	  return SynWait::ESTABLISHED_AS_RESPONDER;
	}
      }
      else if (pureSyn)
      {
	// The peer opened a connection at the same time: handleSyn() applies the tie-break. If this node yields, the
	// SYN is accepted (state SYN_RCVD) and the peer's final ACK is awaited; if it wins, the peer is answered BUSY
	// and our own SYN-ACK is still awaited.
	handleSyn(pkt, hal_->millis());
	if (connection_.state == ConnectionState::SYN_RCVD && connection_.peerAddr == targetAddr)
	{
	  yielded = true;
	  startMs = hal_->millis();
	}
      }
      else if (pkt.header.messageId == msgId)
      {
	if (pkt.header.flags & FLAG_CONN_NACK)
	{
	  stats_.connNacked++;
	  ESP_LOGW(TAG, "Connection NACK received for MsgID %u (Reason code: %u)",
	           msgId, (unsigned)pkt.payload.data[0]);
	  return SynWait::FAILED;
	}

	if ((pkt.header.flags & (FLAG_CONN_REQ | FLAG_CONN_ACK)) == (FLAG_CONN_REQ | FLAG_CONN_ACK))
	{
	  // Loss emulation on the return path (the frame is the one this node was waiting for)
	  if (dropPacketCallback_ && dropPacketCallback_(pkt))
	  {
	    ESP_LOGW(TAG, "[SIMULATED LOSS] Artificially dropping SYN-ACK for MsgID %u", (unsigned)msgId);
	    continue;
	  }
	  if (pkt.header.payloadSize >= sizeof(SynAckMetadata))
	  {
	    std::memcpy(&synAckOut, pkt.payload.data, sizeof(SynAckMetadata));
	    return SynWait::SYN_ACK;
	  }
	}
      }
    }

    hal_->delay(LoRaMultiPacketConfig::POLL_SLEEP_DELAY_MS);
  }

  return SynWait::FAILED;
}

bool LoRaProtocol::send(uint8_t targetAddress, const std::vector<uint8_t> &data, bool reliable)
{
  if (data.empty())
  {
    return false;
  }

  if (reliable && !PacketValidator::canSendReliablyTo(targetAddress))
  {
    ESP_LOGE(TAG, "Reliable send to broadcast rejected: there is no single peer to acknowledge. Use a unicast target.");
    return false;
  }

  uint16_t msgId = allocateMessageId();

  // Negotiated in the handshake when connected to this peer; the maximum otherwise.
  const size_t chunkPayloadSize = connection_.payloadSizeFor(targetAddress);

  std::vector<Packet> packets = PacketSerializer::splitVectorToPackets(data, msgId, nodeAddress_, targetAddress, chunkPayloadSize);
  if (packets.empty())
  {
    return false;
  }

  if (reliable)
  {
    packets.back().header.flags |= FLAG_ACK_REQ;
    packets.back().calculateCRC();
    return sendReliable(packets, chunkPayloadSize);
  }

  return sendUnreliable(packets);
}

bool LoRaProtocol::sendUnreliable(const std::vector<Packet> &packets)
{
  ESP_LOGI(TAG, "Sending MsgID %u (%u chunks, %u bytes total) [Unreliable Mode]",
           packets[0].header.messageId, (unsigned)packets.size(),
           (unsigned)(packets.size() * LORA_MAX_PAYLOAD_SIZE));

  stats_.chunksTx += packets.size();

  for (const auto &packet : packets)
  {
    int state = transmitPacket(packet, "PACKET");
    if (state != RadioStatus::OK)
    {
      ESP_LOGE(TAG, "TX Failed (Chunk %u): %d", packet.header.chunkIndex, state);
      stats_.packetsTxFailed++;
      if (yieldCallback_)
      {
	yieldCallback_();
      }
      return false;
    }
  }

  ESP_LOGI(TAG, "TX Complete.");
  if (yieldCallback_)
  {
    yieldCallback_();
  }

  radio_->startReceive();
  stats_.packetsTx++;
  return true;
}

bool LoRaProtocol::sendReliable(const std::vector<Packet> &packets, size_t chunkPayloadSize)
{
  uint16_t msgId = packets[0].header.messageId;
  uint8_t targetAddr = packets[0].header.dstAddr;
  uint8_t totalChunks = static_cast<uint8_t>(packets.size());
  uint32_t ackTimeoutMs = calculateAckTimeoutMs(totalChunks, chunkPayloadSize);

  ESP_LOGI(TAG, "Sending MsgID %u (%u chunks) [Reliable Mode, Timeout=%ums]",
           msgId, (unsigned)totalChunks, (unsigned)ackTimeoutMs);

  stats_.chunksTx += packets.size();

  for (const auto &packet : packets)
  {
    int state = transmitPacket(packet, "PACKET");
    if (state != RadioStatus::OK)
    {
      ESP_LOGE(TAG, "TX Failed (Chunk %u): %d", packet.header.chunkIndex, state);
      stats_.packetsTxFailed++;
      if (yieldCallback_)
      {
	yieldCallback_();
      }
      return false;
    }
  }

  ESP_LOGI(TAG, "Initial TX Complete. Waiting for SACK...");
  if (yieldCallback_)
  {
    yieldCallback_();
  }

  int retries = 0;
  // Chunk that carried FLAG_ACK_REQ in the last round: the last chunk of the message after the initial burst, the
  // last chunk reported missing after a repair round. A timeout re-sends it, so the poll also repairs it if it was lost.
  uint8_t pollIdx = static_cast<uint8_t>(totalChunks - 1);

  while (true)
  {
    std::vector<uint8_t> sackBitmap;
    if (waitForSack(targetAddr, msgId, ackTimeoutMs, sackBitmap))
    {
      if (connection_.state == ConnectionState::ESTABLISHED && connection_.peerAddr == targetAddr)
      {
	connection_.lastActivityMs = hal_->millis();
      }
      auto missingIndices = SackHelper::getMissingChunkIndices(sackBitmap, totalChunks);
      if (missingIndices.empty())
      {
	ESP_LOGI(TAG, "Reliable send successful! All chunks ACKed.");
	stats_.packetsTx++;
	radio_->startReceive();
	return true;
      }

      ESP_LOGI(TAG, "SACK received. Chunks missing: %u", (unsigned)missingIndices.size());
      // One repair-round budget (R_max) is shared by ACK timeouts and SACK-with-gaps rounds.
      if (retries >= LoRaMultiPacketConfig::MAX_RETRIES)
      {
	ESP_LOGE(TAG, "Reliable send failed: repair round budget exhausted.");
	radio_->startReceive();
	return false;
      }
      if (!retransmitMissingChunks(packets, missingIndices))
      {
	return false;
      }
      pollIdx = missingIndices.back();
      retries++;
      continue;
    }

    int maxRetries = LoRaMultiPacketConfig::MAX_RETRIES;
    if (retries >= maxRetries)
    {
      ESP_LOGE(TAG, "Reliable send failed: Max retries exceeded.");
      radio_->startReceive();
      return false;
    }

    ESP_LOGW(TAG, "ACK timeout. Re-polling with chunk %u (%u/%u)", (unsigned)pollIdx, (unsigned)retries, (unsigned)maxRetries);
    Packet retryPkt = packets[pollIdx];
    retryPkt.header.flags |= FLAG_ACK_REQ;
    retryPkt.calculateCRC();

    stats_.chunksTx++;
    int state = transmitPacket(retryPkt, "RETRY PACKET");
    if (state != RadioStatus::OK)
    {
      ESP_LOGE(TAG, "Retry transmit failed: %d", state);
      stats_.packetsTxFailed++;
      return false;
    }
    radio_->startReceive();
    retries++;
  }
}

uint32_t LoRaProtocol::calculateAckTimeoutMs(size_t totalChunks, size_t chunkPayloadSize) const
{
  size_t maxPacketLen = HEADER_SIZE + chunkPayloadSize + CRC_SIZE;
  uint32_t toaDataMs = radio_->getTimeOnAirUs(maxPacketLen) / 1000;
  size_t sackLen = HEADER_SIZE + ((totalChunks + 7) / 8) + CRC_SIZE;
  uint32_t toaSackMs = radio_->getTimeOnAirUs(sackLen) / 1000;

  // The timer starts after the burst has been sent, so it only has to cover one data frame, the turnaround
  // and the SACK (Eq. timeout in the paper), not the airtime of the whole burst.
  uint32_t timeoutMs = static_cast<uint32_t>(
      LoRaMultiPacketConfig::ACK_TIMEOUT_SAFETY_FACTOR * (toaDataMs + toaSackMs) +
      LoRaMultiPacketConfig::SACK_PREAMBLE_GUARD_DELAY_MS +
      LoRaMultiPacketConfig::ACK_TIMEOUT_GUARD_MS);
  return (timeoutMs < LoRaMultiPacketConfig::MIN_ACK_TIMEOUT_MS)
             ? LoRaMultiPacketConfig::MIN_ACK_TIMEOUT_MS
	     : timeoutMs;
}

bool LoRaProtocol::waitForSack(uint8_t targetAddr, uint16_t msgId, uint32_t timeoutMs, std::vector<uint8_t> &sackBitmapOut)
{
  uint32_t startTime = hal_->millis();

  while (hal_->millis() - startTime < timeoutMs)
  {
    if (yieldCallback_)
    {
      yieldCallback_();
    }

    auto pktOpt = tryReceivePacket();
    if (pktOpt.has_value())
    {
      const auto &packet = pktOpt.value();
      if ((packet.header.flags & FLAG_ACK) && (packet.header.messageId == msgId) && addressedTo(nodeAddress_, packet.header))
      {
	// Edge Case Check: Ignore SACK from unexpected nodes if targetAddr is Unicast
	if (targetAddr != ADDRESS_BROADCAST && packet.header.srcAddr != targetAddr)
	{
	  ESP_LOGW(TAG, "Ignoring SACK for MsgID %u from unexpected node 0x%02X (expected 0x%02X)",
	           msgId, (unsigned)packet.header.srcAddr, (unsigned)targetAddr);
	  continue;
	}

	// Loss emulation on the return path (the frame is the SACK this node was waiting for)
	if (dropPacketCallback_ && dropPacketCallback_(packet))
	{
	  ESP_LOGW(TAG, "[SIMULATED LOSS] Artificially dropping SACK for MsgID %u", (unsigned)msgId);
	  continue;
	}

	if (verbose_)
	{
	  ESP_LOGI(TAG, "<<< DUMPING RX SACK PACKET <<<");
	  packet.printPacket();
	  ESP_LOGI(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	}

	sackBitmapOut.assign(packet.payload.data, packet.payload.data + packet.header.payloadSize);

	if (verbose_)
	{
	  char hexBuf[128] = {0};
	  size_t offset = 0;
	  for (size_t i = 0; i < sackBitmapOut.size() && offset < sizeof(hexBuf) - 10; ++i)
	  {
	    offset += std::snprintf(hexBuf + offset, sizeof(hexBuf) - offset, "0x%02X ", sackBitmapOut[i]);
	  }
	  ESP_LOGI(TAG, "Received SACK for MsgID %u. Bitmap Bytes: %s", msgId, hexBuf);
	}
	return true;
      }
    }

    hal_->delay(LoRaMultiPacketConfig::POLL_SLEEP_DELAY_MS);
  }

  return false;
}

bool LoRaProtocol::retransmitMissingChunks(const std::vector<Packet> &packets,
                                           const std::vector<uint8_t> &missingIndices)
{
  stats_.chunksTx += missingIndices.size();

  for (size_t k = 0; k < missingIndices.size(); ++k)
  {
    uint8_t idx = missingIndices[k];
    Packet pkt = packets[idx];

    if (k == missingIndices.size() - 1)
    {
      pkt.header.flags |= FLAG_ACK_REQ;
    }
    else
    {
      pkt.header.flags &= ~FLAG_ACK_REQ;
    }
    pkt.calculateCRC();

    int state = transmitPacket(pkt, "RETRANSMIT PACKET");
    if (state != RadioStatus::OK)
    {
      ESP_LOGE(TAG, "Retransmit failed (Chunk %u): %d", idx, state);
      stats_.packetsTxFailed++;
      return false;
    }
  }

  radio_->startReceive();
  return true;
}

void LoRaProtocol::update(uint32_t /*currentTimestampMs*/)
{
  // One time base for every timer: the platform clock also used by connect() and sendReliable() to stamp the
  // connection, so that the inactivity check compares values from the same source.
  const uint32_t currentTimestampMs = hal_->millis();

  // A session stays alive while the sender may still be repairing it: up to (R_max + 1) SACK timeouts of silence,
  // never less than PRUNE_TIMEOUT_MS.
  // Duplicates of a completed message can come from the sender's whole retry window, which scales with the SACK
  // timeout of the radio settings in use: the history is kept for 1.2 x that window (never less than the default).
  const uint32_t worstRetryWindowMs = (LoRaMultiPacketConfig::MAX_RETRIES + 1) * calculateAckTimeoutMs(255);
  reassembler_.setCompletedHistoryTtl(
      std::max(LoRaMultiPacketConfig::COMPLETED_HISTORY_TTL_MS, worstRetryWindowMs + worstRetryWindowMs / 5));

  reassembler_.prune(currentTimestampMs,
                     [this](uint8_t totalChunks)
                     {
                       uint32_t retryWindowMs = (LoRaMultiPacketConfig::MAX_RETRIES + 1) * calculateAckTimeoutMs(totalChunks);
                       return std::max(LoRaMultiPacketConfig::PRUNE_TIMEOUT_MS, retryWindowMs);
                     });

  // Inactivity / Idle Timeout Check for active connections
  if (connection_.state == ConnectionState::ESTABLISHED || connection_.state == ConnectionState::SYN_RCVD)
  {
    if (connection_.inactivityExpired(currentTimestampMs))
    {
      ESP_LOGW(TAG, "Connection session MsgID %u TIMED OUT due to inactivity (%u ms). Closing connection.",
               connection_.sessionMsgId, (unsigned)(currentTimestampMs - connection_.lastActivityMs));
      connection_.state = ConnectionState::CLOSED;
    }
  }

  auto pktOpt = tryReceivePacket();
  if (pktOpt.has_value())
  {
    stats_.chunksRx++;
    handleIncomingPacket(pktOpt.value(), currentTimestampMs);
  }
}

std::optional<Packet> LoRaProtocol::tryReceivePacket()
{
  if (!hal_->readIrqPin(irqPin_))
  {
    return std::nullopt;
  }

  uint32_t irqFlags = radio_->getIrqFlags();
  if (verbose_)
  {
    ESP_LOGI(TAG, "IRQ Pin HIGH detected! IrqFlags = 0x%04X", (unsigned)irqFlags);
  }

  if (irqFlags & RadioIrq::RX_DONE)
  {
    radio_->clearIrqFlags(RadioIrq::ALL);
    size_t len = radio_->getPacketLength();
    if (len > 0)
    {
      int state = radio_->readData(phyBuffer_, len);
      if (state == RadioStatus::OK)
      {
	auto packetOpt = PacketParser::parse(phyBuffer_, len);
	if (packetOpt.has_value())
	{
	  radio_->startReceive();
	  return packetOpt;
	}
	else
	{
	  ESP_LOGW(TAG, "Packet Parse Error (CRC/Header mismatch)");
	  stats_.packetsFailed++;
	}
      }
      else
      {
	ESP_LOGE(TAG, "Radio readData failed: %d", state);
	stats_.packetsFailed++;
      }
    }
    else
    {
      ESP_LOGW(TAG, "Ghost Packet detected (len=0), clearing IRQ.");
    }
    radio_->startReceive();
  }
  else
  {
    // Clear residual non-RX IRQ flags (e.g. TX_DONE) so DIO1 pin goes LOW
    radio_->clearIrqFlags(RadioIrq::ALL);
    radio_->startReceive();
  }

  return std::nullopt;
}

void LoRaProtocol::handleSyn(const Packet &packet, uint32_t nowMs)
{
  const uint16_t msgId = packet.header.messageId;
  const uint8_t senderAddr = packet.header.srcAddr;

  stats_.synRcvd++;

  // Edge Case #4: Simultaneous SYN Collision Tie-Breaking
  if (connection_.state == ConnectionState::SYN_SENT && connection_.peerAddr == senderAddr)
  {
    if (nodeAddress_ < senderAddr)
    {
      ESP_LOGW(TAG, "Simultaneous SYN collision: Node 0x%02X wins tie-break over 0x%02X. Replying BUSY.",
               (unsigned)nodeAddress_, (unsigned)senderAddr);
      sendConnNack(senderAddr, msgId, ConnNackReason::BUSY);
      return;
    }
    else
    {
      ESP_LOGI(TAG, "Simultaneous SYN collision: Node 0x%02X recedes to 0x%02X. Accepting peer SYN.",
               (unsigned)nodeAddress_, (unsigned)senderAddr);
    }
  }
  else if (connection_.state != ConnectionState::CLOSED && connection_.peerAddr != senderAddr)
  {
    // Failure Mode: Receiver BUSY in active session with another node
    ESP_LOGW(TAG, "SYN rejected for MsgID %u from 0x%02X: Receiver BUSY with active session (Peer: 0x%02X)",
             msgId, (unsigned)senderAddr, (unsigned)connection_.peerAddr);
    sendConnNack(senderAddr, msgId, ConnNackReason::BUSY);
    return;
  }

  // Failure Mode: Protocol Version Mismatch
  if (packet.header.protocolVersion != LoRaMultiPacketConfig::PROTOCOL_VERSION)
  {
    ESP_LOGE(TAG, "SYN rejected for MsgID %u: Protocol version mismatch (%u != %u)",
             msgId, (unsigned)packet.header.protocolVersion, (unsigned)LoRaMultiPacketConfig::PROTOCOL_VERSION);
    sendConnNack(senderAddr, msgId, ConnNackReason::UNSUPPORTED_VERSION);
    return;
  }

  // Failure Mode: no session slot or heap left. Answer BUSY instead of accepting a transfer we cannot hold.
  auto admission = reassembler_.admitConnection(freeHeapProvider_());
  if (admission != PacketReassembler::Admission::OK)
  {
    ESP_LOGW(TAG, "SYN rejected for MsgID %u from 0x%02X: %s", msgId, (unsigned)senderAddr,
             admission == PacketReassembler::Admission::TABLE_FULL ? "session table full" : "not enough free heap");
    sendConnNack(senderAddr, msgId, ConnNackReason::BUSY);
    return;
  }

  SynMetadata synReq{};
  if (packet.header.payloadSize >= sizeof(SynMetadata))
  {
    std::memcpy(&synReq, packet.payload.data, sizeof(SynMetadata));
  }

  bool accept = true;
  if (onConnRequest_)
  {
    accept = onConnRequest_(synReq);
  }

  // Parameter Negotiation & Clamping (after application callback customization)
  uint8_t acceptedPayload = synReq.requestedPayloadSize;
  if (acceptedPayload == 0 || acceptedPayload > LORA_MAX_PAYLOAD_SIZE)
  {
    acceptedPayload = LORA_MAX_PAYLOAD_SIZE;  // Clamp to maximum supported payload
  }

  uint8_t acceptedWindow = (synReq.windowSize == 0) ? 1 : synReq.windowSize;

  if (accept)
  {
    // A new connection means the sender may have restarted: its old message IDs must not be taken for duplicates
    // and its partial sessions must not absorb the frames of the new message.
    reassembler_.forgetSender(senderAddr);

    connection_.state = ConnectionState::SYN_RCVD;
    connection_.sessionMsgId = msgId;
    connection_.peerAddr = senderAddr;
    connection_.negotiatedPayloadSize = acceptedPayload;
    connection_.windowSize = acceptedWindow;
    connection_.lastActivityMs = nowMs;
    // The peer proposes the inactivity timeout; it decides how long this node's only connection slot stays taken
    // by a peer that goes silent, so it is bounded (0 = no preference).
    uint32_t requestedTimeoutMs = (synReq.timeoutMs > 0) ? synReq.timeoutMs : LoRaMultiPacketConfig::DEFAULT_CONN_INACTIVITY_TIMEOUT_MS;
    connection_.timeoutMs = std::min(std::max(requestedTimeoutMs, LoRaMultiPacketConfig::MIN_CONN_INACTIVITY_TIMEOUT_MS),
                                     LoRaMultiPacketConfig::MAX_CONN_INACTIVITY_TIMEOUT_MS);

    SynAckMetadata synAckResp{};
    synAckResp.acceptedPayloadSize = acceptedPayload;
    synAckResp.windowSize = acceptedWindow;
    synAckResp.reserved = 0;

    ESP_LOGI(TAG, "SYN received for MsgID %u from 0x%02X (ReqPayload=%uB -> AccPayload=%uB). Replying with SYN-ACK...",
             msgId, (unsigned)senderAddr, (unsigned)synReq.requestedPayloadSize, (unsigned)acceptedPayload);
    if (!sendSynAck(senderAddr, msgId, synAckResp))
    {
      ESP_LOGE(TAG, "SYN-ACK for MsgID %u could not be transmitted: connection closed", (unsigned)msgId);
      connection_.state = ConnectionState::CLOSED;
    }
  }
  else
  {
    ESP_LOGW(TAG, "SYN received for MsgID %u from 0x%02X but application REJECTED connection. Sending NACK.", msgId, (unsigned)senderAddr);
    sendConnNack(senderAddr, msgId, ConnNackReason::REJECTED);
  }
}

void LoRaProtocol::handleIncomingPacket(const Packet &packet, uint32_t currentTimestampMs)
{
  if (verbose_)
  {
    ESP_LOGI(TAG, "<<< DUMPING RX PACKET <<<");
    packet.printPacket();
    ESP_LOGI(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>");
  }

  // Self-Echo Drop: Ignore packet if it originates from our own node address
  if (nodeAddress_ != ADDRESS_UNASSIGNED && packet.header.srcAddr == nodeAddress_)
  {
    ESP_LOGI(TAG, "Ignoring self-echoed packet from my own address 0x%02X", (unsigned)nodeAddress_);
    radio_->startReceive();
    return;
  }

  // Address Filtering: drop if packet is not for us, not broadcast, and node address is set
  if (packet.header.dstAddr != nodeAddress_ &&
      packet.header.dstAddr != ADDRESS_BROADCAST &&
      nodeAddress_ != ADDRESS_UNASSIGNED)
  {
    ESP_LOGI(TAG, "Dropping packet not addressed to this node: dstAddr=0x%02X (myAddr=0x%02X, srcAddr=0x%02X)",
             (unsigned)packet.header.dstAddr, (unsigned)nodeAddress_, (unsigned)packet.header.srcAddr);
    stats_.packetsDroppedAddress++;
    radio_->startReceive();
    return;
  }

  // Broadcast frames cannot be acknowledged or open connections: drop them without answering.
  if (PacketValidator::isBroadcastWithReliabilityFlags(packet.header))
  {
    ESP_LOGW(TAG, "Dropping broadcast frame with ACK_REQ/connection flags (0x%02X) from 0x%02X",
             (unsigned)packet.header.flags, (unsigned)packet.header.srcAddr);
    radio_->startReceive();
    return;
  }

  if (dropPacketCallback_ && dropPacketCallback_(packet))
  {
    ESP_LOGW(TAG, "[SIMULATED LOSS] Artificially dropping Packet: MsgID=%u ChunkIndex=%u/%u",
             (unsigned)packet.header.messageId, (unsigned)(packet.header.chunkIndex + 1), (unsigned)packet.header.totalChunks);
    radio_->startReceive();
    return;
  }

  uint16_t msgId = packet.header.messageId;
  uint8_t senderAddr = packet.header.srcAddr;

  // Handle 3-Way Handshake Connection Control Frames
  if (packet.header.flags & FLAG_CONN_REQ)
  {
    if (!(packet.header.flags & FLAG_CONN_ACK))  // Pure SYN packet
    {
      handleSyn(packet, currentTimestampMs);
      return;
    }
  }
  else if (packet.header.flags & FLAG_CONN_ACK)  // Final ACK of Handshake
  {
    if (connection_.state == ConnectionState::SYN_RCVD && connection_.sessionMsgId == msgId && connection_.peerAddr == senderAddr)
    {
      connection_.state = ConnectionState::ESTABLISHED;
      connection_.lastActivityMs = currentTimestampMs;
      stats_.connEstablished++;
      ESP_LOGI(TAG, "3-Way Handshake SUCCESS! Connection ESTABLISHED (Server Mode, MsgID %u).", msgId);
      radio_->startReceive();
      return;
    }
  }

  // A control frame that the handshake logic above did not consume (a SYN-ACK or CONN_NACK after connect() gave up, a
  // CONN_ACK that matches no handshake) is not application data: it must not reach the reassembler.
  if (packet.header.flags & (FLAG_CONN_REQ | FLAG_CONN_ACK | FLAG_CONN_NACK))
  {
    ESP_LOGI(TAG, "Ignoring unexpected control frame (flags 0x%02X, MsgID %u) from 0x%02X",
             (unsigned)packet.header.flags, (unsigned)msgId, (unsigned)senderAddr);
    radio_->startReceive();
    return;
  }

  // Implicit Connection Promotion: if the final CONN_ACK was lost but the peer started sending data, the data frame
  // proves the handshake completed. Data messages carry their own message ID, so the peer address identifies them.
  if (connection_.state == ConnectionState::SYN_RCVD && connection_.peerAddr == senderAddr &&
      !(packet.header.flags & FLAG_ACK))
  {
    ESP_LOGI(TAG, "Data frame received in SYN_RCVD state: Implicitly promoting connection to ESTABLISHED (MsgID %u).", msgId);
    connection_.state = ConnectionState::ESTABLISHED;
    connection_.lastActivityMs = currentTimestampMs;
    stats_.connEstablished++;
  }
  else
  {
    connection_.touchOnFrameFrom(senderAddr, currentTimestampMs);
  }

  if (packet.header.flags & FLAG_ACK)
  {
    ESP_LOGI(TAG, "Received unexpected SACK packet (ignored outside TX loop)");
  }
  else
  {
    bool isAlreadyCompleted = reassembler_.isCompleted(senderAddr, msgId, currentTimestampMs);
    bool justCompleted = false;

    std::optional<std::vector<uint8_t>> payloadOpt;

    if (!isAlreadyCompleted && !reassembler_.isConsistent(packet))
    {
      ESP_LOGW(TAG, "Dropping chunk %u of MsgID %u from 0x%02X: totalChunks=%u does not match the open session",
               (unsigned)packet.header.chunkIndex, (unsigned)msgId, (unsigned)senderAddr,
               (unsigned)packet.header.totalChunks);
      radio_->startReceive();
      return;
    }

    if (!isAlreadyCompleted)
    {
      // Opening a session needs a free slot and enough heap for the whole message. A connectionless message that does
      // not fit is dropped silently: the sender gets no SACK and gives up after its retry budget (a SYN, instead, is
      // answered with CONN_NACK(BUSY), see above).
      auto admission = reassembler_.admit(senderAddr, msgId, packet.header.totalChunks, freeHeapProvider_());
      if (admission != PacketReassembler::Admission::OK)
      {
	ESP_LOGW(TAG, "Refusing MsgID %u from 0x%02X (%u chunks): %s", (unsigned)msgId, (unsigned)senderAddr,
	         (unsigned)packet.header.totalChunks,
	         admission == PacketReassembler::Admission::TABLE_FULL ? "session table full" : "not enough free heap");
	radio_->startReceive();
	return;
      }

      payloadOpt = reassembler_.processPacket(packet, currentTimestampMs);
      if (payloadOpt.has_value())
      {
	reassembler_.markCompleted(senderAddr, msgId, currentTimestampMs);
	stats_.packetsRx++;
	justCompleted = true;
      }
    }

    if (packet.header.flags & FLAG_ACK_REQ)
    {
      bool allReceived = isAlreadyCompleted || justCompleted;
      sendSACK(senderAddr, msgId, packet.header.totalChunks, allReceived);
    }

    if (justCompleted && onReceive_)
    {
      ESP_LOGI(TAG, "Reassembly Complete! (%u bytes from 0x%02X)", (unsigned)payloadOpt.value().size(), (unsigned)senderAddr);
      onReceive_(payloadOpt.value(), radio_->getRSSI(), radio_->getSNR());
    }
  }
}

void LoRaProtocol::sendSACK(uint8_t targetAddr, uint16_t messageId, uint8_t totalChunks, bool allReceived)
{
  std::vector<uint8_t> bitmap;
  if (allReceived)
  {
    bitmap = SackHelper::createFullAckBitmap(totalChunks);
  }
  else
  {
    reassembler_.getReceivedBitmap(targetAddr, messageId, bitmap);
  }

  Packet ackPacket = SackHelper::createSackPacket(messageId, totalChunks, bitmap, nodeAddress_, targetAddr);

  ESP_LOGI(TAG, "Sending SACK for MsgID %u to 0x%02X (Len=%u, AllReceived=%d)",
           messageId, (unsigned)targetAddr, (unsigned)bitmap.size(), allReceived ? 1 : 0);

  hal_->delay(LoRaMultiPacketConfig::SACK_PREAMBLE_GUARD_DELAY_MS);

  int state = transmitPacket(ackPacket, "SACK PACKET");
  if (state != RadioStatus::OK)
  {
    ESP_LOGE(TAG, "Failed to send SACK: %d", state);
  }

  radio_->startReceive();
}

void LoRaProtocol::setOnReceiveCallback(OnReceiveCallback callback)
{
  onReceive_ = callback;
}

void LoRaProtocol::setOnConnectionRequestCallback(OnConnectionRequestCallback callback)
{
  onConnRequest_ = callback;
}

void LoRaProtocol::setYieldCallback(YieldCallback callback)
{
  yieldCallback_ = callback;
}

void LoRaProtocol::setDropPacketCallback(DropPacketCallback callback)
{
  dropPacketCallback_ = callback;
}

void LoRaProtocol::setFreeHeapProvider(FreeHeapProvider provider)
{
  freeHeapProvider_ = provider ? std::move(provider) : FreeHeapProvider(defaultFreeHeap);
}

void LoRaProtocol::setVerbose(bool enable)
{
  verbose_ = enable;
}

int LoRaProtocol::transmitPacket(const Packet &packet, const char *logPrefix)
{
  if (yieldCallback_)
  {
    yieldCallback_();
  }

  const size_t len = PacketSerializer::serialize(packet, phyBuffer_, sizeof(phyBuffer_));
  if (len == 0)
  {
    ESP_LOGE(TAG, "Refusing to transmit a frame that does not fit the PHY buffer (payloadSize=%u)",
             (unsigned)packet.header.payloadSize);
    return RadioStatus::FAILED;
  }

  if (verbose_)
  {
    ESP_LOGI(TAG, ">>> DUMPING TX %s >>>", logPrefix);
    packet.printPacket();
    ESP_LOGI(TAG, "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  }

  int state = radio_->transmit(phyBuffer_, len);
  radio_->clearIrqFlags(RadioIrq::ALL);
  radio_->startReceive();
  if (state == RadioStatus::OK)
  {
    hal_->delay(LoRaMultiPacketConfig::POST_TX_GUARD_DELAY_MS);
  }
  return state;
}