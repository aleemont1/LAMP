#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "Packet.hpp"
#include "ProtocolConfig.hpp"

/**
 * @class PacketReassembler
 * @brief Manages the reconstruction of split messages from individual Packet chunks.
 *
 * This class handles:
 * - Storage of partial message fragments.
 * - Out-of-order packet insertion.
 * - Reassembly of complete messages.
 * - Timeout-based cleanup of incomplete stale messages.
 */
class PacketReassembler
{
 public:
  /**
   * @brief Processes an incoming packet and attempts to reassemble the full message.
   *
   * If the packet completes a sequence, the full payload is returned.
   * If the sequence is still incomplete, std::nullopt is returned.
   *
   * @param packet The valid packet received from the network.
   * @param currentTimestampMs A distinct timestamp (e.g., millis) to track timeout.
   * @return std::optional<std::vector<uint8_t>> The complete reassembled payload if finished.
   */
  std::optional<std::vector<uint8_t>> processPacket(const Packet &packet, uint32_t currentTimestampMs);

  /**
   * @brief Checks that a frame is consistent with the open session of its sender and message ID, if there is one.
   *
   * A frame that declares a different totalChunks than the session was opened with, or whose chunk index falls
   * outside the session, cannot belong to that message (e.g. a sender that rebooted and reused the message ID).
   * processPacket() drops such frames; callers that react to a frame (SACK, statistics) should check this first.
   *
   * @return true if there is no open session for the frame, or the frame matches it.
   */
  bool isConsistent(const Packet &packet) const;

  /**
   * @brief Removes incomplete messages that have been inactive for longer than the timeout.
   *
   * Inactivity is measured from the last chunk received for the message (duplicates included), so a sender that is
   * still repairing a transfer keeps its session alive. Should be called periodically to free up memory from
   * abandoned sequences.
   *
   * @param currentTimestampMs The current system time.
   * @param timeoutMs The maximum inactivity before an incomplete message is dropped.
   */
  void prune(uint32_t currentTimestampMs, uint32_t timeoutMs);

  /**
   * @brief Same as prune(now, timeoutMs), with an inactivity limit that depends on the message's chunk count
   * (the sender's SACK timeout grows with the number of chunks).
   */
  void prune(uint32_t currentTimestampMs, const std::function<uint32_t(uint8_t totalChunks)> &timeoutFor);

  /**
   * @brief Clears all pending reassembly sessions.
   */
  void reset();

  /**
   * @brief Generates a bitmap of successfully received chunks for a given message.
   *
   * @param srcAddr Sender address (sessions are keyed by sender and message ID).
   * @param messageId The message ID of the session.
   * @param bitmapOut Output vector to write the bitmap.
   * @return true if session was found, false otherwise.
   */
  bool getReceivedBitmap(uint8_t srcAddr, uint16_t messageId, std::vector<uint8_t> &bitmapOut) const;

  /**
   * @brief Checks if a message from a given sender was completed within COMPLETED_HISTORY_TTL_MS.
   *
   * An older entry is not a duplicate any more: the sender may have rebooted and reused the message ID.
   */
  bool isCompleted(uint8_t srcAddr, uint16_t messageId, uint32_t currentTimestampMs) const;

  /**
   * @brief Sets how long a completed message keeps suppressing duplicates (default: COMPLETED_HISTORY_TTL_MS).
   *
   * It has to cover the sender's whole retry window, which depends on the radio settings; the protocol sets it from
   * the SACK timeout of the largest message.
   */
  void setCompletedHistoryTtl(uint32_t ttlMs) { completedTtlMs_ = ttlMs; }

  /**
   * @brief Marks a message from a given sender as completed, adding it to the history with its timestamp.
   */
  void markCompleted(uint8_t srcAddr, uint16_t messageId, uint32_t currentTimestampMs);

  /// Outcome of an admission check.
  enum class Admission : uint8_t
  {
    OK,
    TABLE_FULL,  ///< every session slot is in use
    NO_MEMORY    ///< the free heap cannot hold the session plus the margin
  };

  /**
   * @brief Heap needed to reassemble a message of totalChunks chunks: the chunks as stored in the session map plus
   * the reassembled message built at completion.
   */
  static size_t sessionBytes(uint8_t totalChunks);

  /**
   * @brief Decides whether a frame may open a new session, given the free heap.
   *
   * A frame of an already open session is always OK (its memory is accounted for). Otherwise a slot must be free and
   * the heap must hold sessionBytes(totalChunks) plus RX_HEAP_MARGIN_BYTES.
   */
  Admission admit(uint8_t srcAddr, uint16_t messageId, uint8_t totalChunks, size_t freeHeapBytes) const;

  /**
   * @brief Decides whether a connection request may be accepted. The SYN does not carry the message size, so only a
   * free session slot and the heap margin are checked; the size is checked by admit() on the first data frame.
   */
  Admission admitConnection(size_t freeHeapBytes) const;

  /**
   * @brief Forgets everything known about a sender: its completed messages and its partial sessions.
   *
   * Called when the sender opens a new connection, which means it may have restarted and reuse old message IDs.
   */
  void forgetSender(uint8_t srcAddr);

 private:
  static constexpr size_t MAX_COMPLETED_HISTORY = LoRaMultiPacketConfig::MAX_COMPLETED_HISTORY;
  static constexpr size_t MAX_CONCURRENT_MESSAGES = LoRaMultiPacketConfig::MAX_CONCURRENT_SESSIONS;

  /**
   * @brief Keep track of the received chunks for each msgId, with other metadata.
   */
  struct ReassemblySession
  {
    uint8_t totalChunks;
    uint32_t lastReceivedTime;
    std::map<uint8_t, Packet> chunks;

    ReassemblySession(uint8_t total, uint32_t time)
        : totalChunks(total),
          lastReceivedTime(time)
    {
    }
  };

  /// Session key: sender address and message ID (message IDs are only unique per sender).
  static uint32_t sessionKey(uint8_t srcAddr, uint16_t messageId)
  {
    return (static_cast<uint32_t>(srcAddr) << 16) | messageId;
  }

  std::map<uint32_t, ReassemblySession> sessions_;

  struct CompletedEntry
  {
    uint32_t key;
    uint32_t completedTime;
  };
  std::vector<CompletedEntry> completedMessages_;
  uint32_t completedTtlMs_ = LoRaMultiPacketConfig::COMPLETED_HISTORY_TTL_MS;

  static std::vector<uint8_t> reconstruct(const ReassemblySession &session);
};