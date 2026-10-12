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
// How long a completed message keeps suppressing duplicates. A duplicate can only come from the sender's retry
// window after a lost SACK: (MAX_RETRIES + 1) SACK timeouts, about 100 s for a full-size frame at SF12/125 kHz
// (SACK timeout 16.8 s). Rounded up; raise it for narrower bandwidths or a larger MAX_RETRIES.
constexpr uint32_t COMPLETED_HISTORY_TTL_MS = 120000;

// Airtime Safety Multipliers
constexpr double ACK_TIMEOUT_SAFETY_FACTOR = 1.5;
constexpr uint32_t ACK_TIMEOUT_GUARD_MS = 25;

// Buffer Sizes
constexpr size_t PHY_BUFFER_SIZE = 256;
}  // namespace LoRaMultiPacketConfig
