#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @namespace LoRaMultiPacketConfig
 * @brief Centralized configuration constants for the LoRaMultiPacket protocol.
 */
namespace LoRaMultiPacketConfig
{
// Protocol Version
constexpr uint8_t PROTOCOL_VERSION = 2;

// Connection and Handshake Configuration
constexpr uint32_t DEFAULT_SYN_TIMEOUT_MS = 3000;
constexpr uint32_t DEFAULT_CONN_INACTIVITY_TIMEOUT_MS = 15000;
// Bounds applied to the inactivity timeout a peer asks for in its SYN: it decides how long the single connection slot
// of this node stays taken by a peer that goes silent.
constexpr uint32_t MIN_CONN_INACTIVITY_TIMEOUT_MS = 3000;
constexpr uint32_t MAX_CONN_INACTIVITY_TIMEOUT_MS = 30000;

// Retry and Timeout Limits
constexpr int MAX_RETRIES = 5;
constexpr uint32_t MIN_ACK_TIMEOUT_MS = 1000;
constexpr uint32_t PRUNE_TIMEOUT_MS = 15000;  // minimum reassembly inactivity before an incomplete message is dropped

// Delays (in milliseconds)
constexpr uint32_t SACK_PREAMBLE_GUARD_DELAY_MS = 25;
constexpr uint32_t POST_TX_GUARD_DELAY_MS = 5;
constexpr uint32_t POLL_SLEEP_DELAY_MS = 10;

// Session and Reassembly Limits
constexpr size_t MAX_CONCURRENT_SESSIONS = 10;
constexpr size_t MAX_COMPLETED_HISTORY = 16;

// Receiver heap admission. A reassembly session holds one std::map node per received chunk (a Packet plus the node
// bookkeeping) and, once complete, the reassembled message next to them.
constexpr size_t RX_CHUNK_NODE_OVERHEAD_BYTES = 32;  // std::map node bookkeeping per chunk (32-bit target)
constexpr size_t RX_HEAP_MARGIN_BYTES = 16384;       // heap that must stay free for the rest of the firmware
// How long a completed message keeps suppressing duplicates. A duplicate can only come from the sender's retry
// window after a lost SACK: (MAX_RETRIES + 1) SACK timeouts, about 100 s for a full-size frame at SF12/125 kHz
// (SACK timeout 16.8 s). This is the floor: LoRaProtocol raises the TTL to 1.2 x (MAX_RETRIES + 1) SACK timeouts of the
// largest message for the radio settings in use, so narrower bandwidths need no manual change.
constexpr uint32_t COMPLETED_HISTORY_TTL_MS = 120000;

// Airtime Safety Multipliers
constexpr double ACK_TIMEOUT_SAFETY_FACTOR = 1.5;
constexpr uint32_t ACK_TIMEOUT_GUARD_MS = 25;

// Buffer Sizes
constexpr size_t PHY_BUFFER_SIZE = 256;
}  // namespace LoRaMultiPacketConfig
