#include "PacketReassembler.hpp"

#include <algorithm>

#include "PacketDeserializer.hpp"

bool PacketReassembler::isConsistent(const Packet &packet) const
{
  auto it = sessions_.find(sessionKey(packet.header.srcAddr, packet.header.messageId));
  if (it == sessions_.end())
  {
    return true;
  }

  const ReassemblySession &session = it->second;
  if (session.totalChunks != packet.header.totalChunks || packet.header.chunkIndex >= session.totalChunks)
  {
    return false;
  }

  // Chunk sizes: all non-final chunks of a message have the size negotiated in the handshake, and the final one is
  // not larger. The (stateless) validator cannot check this, so the first chunk received sets the stride.
  const uint8_t lastIdx = session.totalChunks - 1;
  size_t stride = 0;  // 0 = not known yet
  for (const auto &entry : session.chunks)
  {
    if (entry.first != lastIdx)
    {
      stride = entry.second.header.payloadSize;
      break;
    }
  }
  if (packet.header.chunkIndex != lastIdx)
  {
    if (stride == 0)
    {
      stride = packet.header.payloadSize;
    }
    else if (packet.header.payloadSize != stride)
    {
      return false;
    }
    auto last = session.chunks.find(lastIdx);
    return last == session.chunks.end() || last->second.header.payloadSize <= stride;
  }
  return stride == 0 || packet.header.payloadSize <= stride;
}

std::optional<std::vector<uint8_t>> PacketReassembler::processPacket(const Packet &packet, uint32_t currentTimestampMs)
{
  // Drop without touching the session: an inconsistent frame must not be stored nor refresh the inactivity timer.
  if (!isConsistent(packet))
  {
    return std::nullopt;
  }

  uint32_t key = sessionKey(packet.header.srcAddr, packet.header.messageId);
  uint8_t chunkIdx = packet.header.chunkIndex;
  uint8_t total = packet.header.totalChunks;

  auto it = sessions_.find(key);
  if (it == sessions_.end())
  {
    if (sessions_.size() >= MAX_CONCURRENT_MESSAGES)
    {
      return std::nullopt;
    }
    it = sessions_.emplace(key, ReassemblySession(total, currentTimestampMs)).first;
  }

  ReassemblySession &session = it->second;
  session.lastReceivedTime = currentTimestampMs;
  session.chunks.emplace(chunkIdx, packet);

  if (session.chunks.size() == session.totalChunks)
  {
    std::vector<uint8_t> result = reconstruct(session);
    sessions_.erase(it);
    return result;
  }

  return std::nullopt;
}

void PacketReassembler::prune(uint32_t currentTimestampMs, uint32_t timeoutMs)
{
  prune(currentTimestampMs, [timeoutMs](uint8_t)
        { return timeoutMs; });
}

void PacketReassembler::prune(uint32_t currentTimestampMs, const std::function<uint32_t(uint8_t)> &timeoutFor)
{
  auto it = sessions_.begin();
  while (it != sessions_.end())
  {
    if (currentTimestampMs - it->second.lastReceivedTime > timeoutFor(it->second.totalChunks))
    {
      it = sessions_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void PacketReassembler::reset()
{
  sessions_.clear();
  completedMessages_.clear();
}

bool PacketReassembler::getReceivedBitmap(uint8_t srcAddr, uint16_t messageId, std::vector<uint8_t> &bitmapOut) const
{
  auto it = sessions_.find(sessionKey(srcAddr, messageId));
  if (it == sessions_.end())
  {
    return false;
  }

  const auto &session = it->second;
  size_t bitmapSize = (session.totalChunks + 7) / 8;
  bitmapOut.assign(bitmapSize, 0);

  for (uint8_t i = 0; i < session.totalChunks; ++i)
  {
    if (session.chunks.find(i) != session.chunks.end())
    {
      bitmapOut[i / 8] |= (1 << (i % 8));
    }
  }
  return true;
}

bool PacketReassembler::isCompleted(uint8_t srcAddr, uint16_t messageId, uint32_t currentTimestampMs) const
{
  uint32_t key = sessionKey(srcAddr, messageId);
  return std::any_of(completedMessages_.begin(), completedMessages_.end(),
                     [&](const CompletedEntry &entry)
                     {
                       return entry.key == key &&
		              currentTimestampMs - entry.completedTime <= completedTtlMs_;
                     });
}

void PacketReassembler::markCompleted(uint8_t srcAddr, uint16_t messageId, uint32_t currentTimestampMs)
{
  uint32_t key = sessionKey(srcAddr, messageId);
  // Refresh the entry if it is already there (expired or not), so the key appears once.
  completedMessages_.erase(std::remove_if(completedMessages_.begin(), completedMessages_.end(),
                                          [&](const CompletedEntry &entry)
                                          { return entry.key == key; }),
                           completedMessages_.end());
  if (completedMessages_.size() >= MAX_COMPLETED_HISTORY)
  {
    completedMessages_.erase(completedMessages_.begin());
  }
  completedMessages_.push_back({key, currentTimestampMs});
}

size_t PacketReassembler::sessionBytes(uint8_t totalChunks)
{
  return static_cast<size_t>(totalChunks) *
         (sizeof(Packet) + LoRaMultiPacketConfig::RX_CHUNK_NODE_OVERHEAD_BYTES + LORA_MAX_PAYLOAD_SIZE);
}

PacketReassembler::Admission PacketReassembler::admit(uint8_t srcAddr, uint16_t messageId, uint8_t totalChunks,
                                                      size_t freeHeapBytes) const
{
  if (sessions_.count(sessionKey(srcAddr, messageId)) != 0)
  {
    return Admission::OK;
  }
  if (sessions_.size() >= MAX_CONCURRENT_MESSAGES)
  {
    return Admission::TABLE_FULL;
  }
  if (freeHeapBytes < sessionBytes(totalChunks) + LoRaMultiPacketConfig::RX_HEAP_MARGIN_BYTES)
  {
    return Admission::NO_MEMORY;
  }
  return Admission::OK;
}

PacketReassembler::Admission PacketReassembler::admitConnection(size_t freeHeapBytes) const
{
  if (sessions_.size() >= MAX_CONCURRENT_MESSAGES)
  {
    return Admission::TABLE_FULL;
  }
  return (freeHeapBytes < LoRaMultiPacketConfig::RX_HEAP_MARGIN_BYTES) ? Admission::NO_MEMORY : Admission::OK;
}

void PacketReassembler::forgetSender(uint8_t srcAddr)
{
  auto fromSender = [srcAddr](uint32_t key)
  { return static_cast<uint8_t>(key >> 16) == srcAddr; };

  completedMessages_.erase(std::remove_if(completedMessages_.begin(), completedMessages_.end(),
                                          [&](const CompletedEntry &entry)
                                          { return fromSender(entry.key); }),
                           completedMessages_.end());
  for (auto it = sessions_.begin(); it != sessions_.end();)
  {
    it = fromSender(it->first) ? sessions_.erase(it) : std::next(it);
  }
}

std::vector<uint8_t> PacketReassembler::reconstruct(const ReassemblySession &session)
{
  std::vector<uint8_t> fullMessage;

  // Pre-allocate memory assuming a possible full payload (no dummy bytes).
  fullMessage.reserve(session.totalChunks * LORA_MAX_PAYLOAD_SIZE);

  for (uint8_t i = 0; i < session.totalChunks; ++i)
  {
    auto it = session.chunks.find(i);
    if (it != session.chunks.end())
    {
      PacketDeserializer::deserialize(it->second, fullMessage);
    }
  }

  return fullMessage;
}
