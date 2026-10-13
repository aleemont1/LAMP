#include "LoRaProtocol.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#ifdef ESP_PLATFORM
#include "esp_log.h"
static const char *TAG = "LoRaProto";
#else
#define ESP_LOGI(tag, format, ...) printf("INFO  [%s]: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("WARN  [%s]: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) printf("ERROR [%s]: " format "\n", tag, ##__VA_ARGS__)
static const char *TAG = "LoRaProto";
#endif

LoRaProtocol::LoRaProtocol(SX1262 *radio, RadioLibHal *hal, uint32_t irqPin)
    : radio_(radio),
      hal_(hal),
      irqPin_(irqPin),
      nodeAddress_(ADDRESS_UNASSIGNED),
      onConnRequest_(nullptr),
      dropPacketCallback_(nullptr),
      verbose_(false),
      nextMessageId_(1),
      phyBuffer_{0}
{
  connection_.state = ConnectionState::CLOSED;
}

bool LoRaProtocol::connect(uint8_t targetAddress, uint32_t timeoutMs)
{
  uint16_t msgId = nextMessageId_++;
  if (nextMessageId_ == 0)
  {
    nextMessageId_ = 1;
  }

  connection_.state = ConnectionState::SYN_SENT;
  connection_.sessionMsgId = msgId;
  connection_.peerAddr = targetAddress;
  stats_.synSent++;

  SynMetadata synReq{};
  synReq.requestedPayloadSize = LORA_MAX_PAYLOAD_SIZE;
  synReq.windowSize = 1;
  synReq.timeoutMs = static_cast<uint16_t>(LoRaMultiPacketConfig::DEFAULT_CONN_INACTIVITY_TIMEOUT_MS);

  ESP_LOGI(TAG, "Initiating 3-Way Handshake (MsgID %u, SYN sent to 0x%02X)...", msgId, (unsigned)targetAddress);
  sendSyn(targetAddress, msgId, synReq);

  SynAckMetadata synAckResp{};
  if (!waitForSynAck(targetAddress, msgId, timeoutMs, synAckResp))
  {
    ESP_LOGE(TAG, "3-Way Handshake Failed (SYN-ACK Timeout/NACK for MsgID %u)", msgId);
    connection_.state = ConnectionState::CLOSED;
    return false;
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

  transmitPacket(ackPacket, "CONN ACK PACKET");

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

void LoRaProtocol::sendSyn(uint8_t targetAddr, uint16_t msgId, const SynMetadata &syn)
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

  transmitPacket(p, "SYN PACKET");
}

void LoRaProtocol::sendSynAck(uint8_t targetAddr, uint16_t msgId, const SynAckMetadata &synAck)
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
  transmitPacket(p, "SYN-ACK PACKET");
}

void LoRaProtocol::sendConnNack(uint8_t targetAddr, uint16_t msgId, ConnNackReason reason)
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
  transmitPacket(p, "CONN-NACK PACKET");
}

bool LoRaProtocol::waitForSynAck(uint8_t targetAddr, uint16_t msgId, uint32_t timeoutMs, SynAckMetadata &synAckOut)
{
  uint32_t startMs = hal_->millis();

  while (hal_->millis() - startMs < timeoutMs)
  {
    auto pktOpt = tryReceivePacket();
    if (pktOpt.has_value())
    {
      const Packet &pkt = pktOpt.value();
      if (pkt.header.messageId == msgId)
      {
	// Edge Case Check: Ignore responses from unexpected nodes if targetAddr is Unicast
	if (targetAddr != ADDRESS_BROADCAST && pkt.header.srcAddr != targetAddr)
	{
	  ESP_LOGW(TAG, "Ignoring SYN-ACK for MsgID %u from unexpected node 0x%02X (expected 0x%02X)",
	           msgId, (unsigned)pkt.header.srcAddr, (unsigned)targetAddr);
	  continue;
	}

	if (pkt.header.flags & FLAG_CONN_NACK)
	{
	  stats_.connNacked++;
	  ESP_LOGW(TAG, "Connection NACK received for MsgID %u (Reason code: %u)",
	           msgId, (unsigned)pkt.payload.data[0]);
	  return false;
	}

	if ((pkt.header.flags & (FLAG_CONN_REQ | FLAG_CONN_ACK)) == (FLAG_CONN_REQ | FLAG_CONN_ACK))
	{
	  if (pkt.header.payloadSize >= sizeof(SynAckMetadata))
	  {
	    std::memcpy(&synAckOut, pkt.payload.data, sizeof(SynAckMetadata));
	    return true;
	  }
	}
      }
    }

    hal_->delay(LoRaMultiPacketConfig::POLL_SLEEP_DELAY_MS);
  }

  return false;
}

bool LoRaProtocol::send(uint8_t targetAddress, const std::vector<uint8_t> &data, bool reliable)
{
  if (data.empty())
  {
    return false;
  }

  uint16_t msgId = nextMessageId_++;
  if (nextMessageId_ == 0)
  {
    nextMessageId_ = 1;
  }

  std::vector<Packet> packets = PacketSerializer::splitVectorToPackets(data, msgId, nodeAddress_, targetAddress);
  if (packets.empty())
  {
    return false;
  }

  if (reliable)
  {
    packets.back().header.flags |= FLAG_ACK_REQ;
    packets.back().calculateCRC();
    return sendReliable(packets);
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
    if (state != RADIOLIB_ERR_NONE)
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

bool LoRaProtocol::sendReliable(const std::vector<Packet> &packets)
{
  uint16_t msgId = packets[0].header.messageId;
  uint8_t targetAddr = packets[0].header.dstAddr;
  uint8_t totalChunks = static_cast<uint8_t>(packets.size());
  uint32_t ackTimeoutMs = calculateAckTimeoutMs(totalChunks);

  ESP_LOGI(TAG, "Sending MsgID %u (%u chunks) [Reliable Mode, Timeout=%ums]",
           msgId, (unsigned)totalChunks, (unsigned)ackTimeoutMs);

  stats_.chunksTx += packets.size();

  for (const auto &packet : packets)
  {
    int state = transmitPacket(packet, "PACKET");
    if (state != RADIOLIB_ERR_NONE)
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

    ESP_LOGW(TAG, "ACK timeout. Retrying last chunk (%u/%u)", (unsigned)retries, (unsigned)maxRetries);
    Packet retryPkt = packets[totalChunks - 1];
    retryPkt.header.flags |= FLAG_ACK_REQ;
    retryPkt.calculateCRC();

    stats_.chunksTx++;
    int state = transmitPacket(retryPkt, "RETRY PACKET");
    if (state != RADIOLIB_ERR_NONE)
    {
      ESP_LOGE(TAG, "Retry transmit failed: %d", state);
      stats_.packetsTxFailed++;
      return false;
    }
    radio_->startReceive();
    retries++;
  }
}

uint32_t LoRaProtocol::calculateAckTimeoutMs(size_t totalChunks) const
{
  size_t maxPacketLen = HEADER_SIZE + LORA_MAX_PAYLOAD_SIZE + CRC_SIZE;
  uint32_t toaDataMs = radio_->getTimeOnAir(maxPacketLen) / 1000;
  size_t sackLen = HEADER_SIZE + ((totalChunks + 7) / 8) + CRC_SIZE;
  uint32_t toaSackMs = radio_->getTimeOnAir(sackLen) / 1000;

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
      if ((packet.header.flags & FLAG_ACK) && (packet.header.messageId == msgId))
      {
	// Edge Case Check: Ignore SACK from unexpected nodes if targetAddr is Unicast
	if (targetAddr != ADDRESS_BROADCAST && packet.header.srcAddr != targetAddr)
	{
	  ESP_LOGW(TAG, "Ignoring SACK for MsgID %u from unexpected node 0x%02X (expected 0x%02X)",
	           msgId, (unsigned)packet.header.srcAddr, (unsigned)targetAddr);
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
    if (state != RADIOLIB_ERR_NONE)
    {
      ESP_LOGE(TAG, "Retransmit failed (Chunk %u): %d", idx, state);
      stats_.packetsTxFailed++;
      return false;
    }
  }

  radio_->startReceive();
  return true;
}

void LoRaProtocol::update(uint32_t currentTimestampMs)
{
  // A session stays alive while the sender may still be repairing it: up to (R_max + 1) SACK timeouts of silence,
  // never less than PRUNE_TIMEOUT_MS.
  reassembler_.prune(currentTimestampMs,
                     [this](uint8_t totalChunks)
                     {
                       uint32_t retryWindowMs = (LoRaMultiPacketConfig::MAX_RETRIES + 1) * calculateAckTimeoutMs(totalChunks);
                       return std::max(LoRaMultiPacketConfig::PRUNE_TIMEOUT_MS, retryWindowMs);
                     });

  // Inactivity / Idle Timeout Check for active connections
  if (connection_.state == ConnectionState::ESTABLISHED || connection_.state == ConnectionState::SYN_RCVD)
  {
    if (currentTimestampMs - connection_.lastActivityMs > connection_.timeoutMs)
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
  if (!hal_->digitalRead(irqPin_))
  {
    return std::nullopt;
  }

  uint32_t irqFlags = radio_->getIrqFlags();
  if (verbose_)
  {
    ESP_LOGI(TAG, "IRQ Pin HIGH detected! IrqFlags = 0x%04X", (unsigned)irqFlags);
  }

  if (irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE)
  {
    radio_->clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    size_t len = radio_->getPacketLength();
    if (len > 0)
    {
      int state = radio_->readData(phyBuffer_, len);
      if (state == RADIOLIB_ERR_NONE)
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
    radio_->clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    radio_->startReceive();
  }

  return std::nullopt;
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
	connection_.state = ConnectionState::SYN_RCVD;
	connection_.sessionMsgId = msgId;
	connection_.peerAddr = senderAddr;
	connection_.negotiatedPayloadSize = acceptedPayload;
	connection_.windowSize = acceptedWindow;
	connection_.lastActivityMs = currentTimestampMs;
	connection_.timeoutMs = (synReq.timeoutMs > 0) ? synReq.timeoutMs : LoRaMultiPacketConfig::DEFAULT_CONN_INACTIVITY_TIMEOUT_MS;

	SynAckMetadata synAckResp{};
	synAckResp.acceptedPayloadSize = acceptedPayload;
	synAckResp.windowSize = acceptedWindow;
	synAckResp.reserved = 0;

	ESP_LOGI(TAG, "SYN received for MsgID %u from 0x%02X (ReqPayload=%uB -> AccPayload=%uB). Replying with SYN-ACK...",
	         msgId, (unsigned)senderAddr, (unsigned)synReq.requestedPayloadSize, (unsigned)acceptedPayload);
	sendSynAck(senderAddr, msgId, synAckResp);
      }
      else
      {
	ESP_LOGW(TAG, "SYN received for MsgID %u from 0x%02X but application REJECTED connection. Sending NACK.", msgId, (unsigned)senderAddr);
	sendConnNack(senderAddr, msgId, ConnNackReason::REJECTED);
      }
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

  // Edge Case #3: Implicit Connection Promotion if final CONN_ACK was lost but peer started sending data
  if (connection_.state == ConnectionState::SYN_RCVD && connection_.sessionMsgId == msgId && connection_.peerAddr == senderAddr)
  {
    ESP_LOGI(TAG, "Data frame received in SYN_RCVD state: Implicitly promoting connection to ESTABLISHED (MsgID %u).", msgId);
    connection_.state = ConnectionState::ESTABLISHED;
    connection_.lastActivityMs = currentTimestampMs;
    stats_.connEstablished++;
  }
  else if (connection_.state == ConnectionState::ESTABLISHED && connection_.sessionMsgId == msgId && connection_.peerAddr == senderAddr)
  {
    connection_.lastActivityMs = currentTimestampMs;
  }

  if (packet.header.flags & FLAG_ACK)
  {
    ESP_LOGI(TAG, "Received unexpected SACK packet (ignored outside TX loop)");
  }
  else
  {
    bool isAlreadyCompleted = reassembler_.isCompleted(senderAddr, msgId);
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
      payloadOpt = reassembler_.processPacket(packet, currentTimestampMs);
      if (payloadOpt.has_value())
      {
	reassembler_.markCompleted(senderAddr, msgId);
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
  if (state != RADIOLIB_ERR_NONE)
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

  PacketSerializer::serialize(packet, phyBuffer_);
  size_t len = HEADER_SIZE + packet.header.payloadSize + CRC_SIZE;

  if (verbose_)
  {
    ESP_LOGI(TAG, ">>> DUMPING TX %s >>>", logPrefix);
    packet.printPacket();
    ESP_LOGI(TAG, "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  }

  int state = radio_->transmit(phyBuffer_, len);
  radio_->clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  radio_->startReceive();
  if (state == RADIOLIB_ERR_NONE)
  {
    hal_->delay(LoRaMultiPacketConfig::POST_TX_GUARD_DELAY_MS);
  }
  return state;
}