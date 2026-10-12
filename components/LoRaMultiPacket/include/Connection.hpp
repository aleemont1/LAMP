#pragma once

#include <cstdint>

#include "Packet.hpp"

/**
 * @enum ConnectionState
 * @brief Represents the lifecycle states of a 3-way handshake connection session.
 */
enum class ConnectionState : uint8_t
{
  CLOSED = 0,   ///< No connection established.
  SYN_SENT,     ///< Initiator sent SYN (CONN_REQ), awaiting SYN-ACK (CONN_REQ | CONN_ACK).
  SYN_RCVD,     ///< Receiver got SYN, sent SYN-ACK, awaiting final ACK (CONN_ACK).
  ESTABLISHED,  ///< Handshake complete, active bidirectional connection session.
  FIN_WAIT      ///< Disconnect initiated, awaiting final acknowledgment.
};

/**
 * @enum ConnNackReason
 * @brief Reason codes sent in a CONN_NACK response payload.
 */
enum class ConnNackReason : uint8_t
{
  UNKNOWN = 0,
  BUSY = 1,                 ///< Receiver cannot accept new connection (session table full).
  UNSUPPORTED_VERSION = 2,  ///< Protocol version mismatch.
  REJECTED = 3              ///< Connection explicitly rejected by application callback.
};

#pragma pack(push, 1)

/**
 * @struct SynMetadata
 * @brief Handshake parameters embedded in the payload of a SYN (CONN_REQ) packet.
 * Size: 4 bytes.
 */
struct SynMetadata
{
  uint8_t requestedPayloadSize = LORA_MAX_PAYLOAD_SIZE;  ///< Maximum requested payload bytes per chunk.
  uint8_t windowSize = 1;                                ///< Sliding window size (chunks).
  uint16_t timeoutMs = 5000;                             ///< Maximum inactivity timeout requested.
};

/**
 * @struct SynAckMetadata
 * @brief Handshake parameters embedded in the payload of a SYN-ACK (CONN_REQ | CONN_ACK) packet.
 * Size: 4 bytes.
 */
struct SynAckMetadata
{
  uint8_t acceptedPayloadSize = LORA_MAX_PAYLOAD_SIZE;  ///< Negotiated payload bytes per chunk.
  uint8_t windowSize = 1;                               ///< Negotiated window size.
  uint16_t reserved = 0;                                ///< Reserved alignment padding.
};

#pragma pack(pop)

/**
 * @struct ConnectionSession
 * @brief Represents active connection state and parameters for a peer session.
 */
struct ConnectionSession
{
  ConnectionState state = ConnectionState::CLOSED;
  uint16_t sessionMsgId = 0;
  uint8_t peerAddr = ADDRESS_UNASSIGNED;  ///< Peer node address for active session
  uint8_t negotiatedPayloadSize = LORA_MAX_PAYLOAD_SIZE;
  uint8_t windowSize = 1;
  uint32_t lastActivityMs = 0;
  uint32_t timeoutMs = 15000;

  /**
   * @brief Refreshes the inactivity timer on a frame from the peer of an ESTABLISHED connection.
   *
   * Data messages carry their own message ID (not the handshake's), so the peer address alone identifies traffic
   * that belongs to the connection.
   * @return true if the frame refreshed the connection.
   */
  bool touchOnFrameFrom(uint8_t srcAddr, uint32_t nowMs)
  {
    if (state != ConnectionState::ESTABLISHED || peerAddr != srcAddr)
    {
      return false;
    }
    lastActivityMs = nowMs;
    return true;
  }

  /**
   * @brief True when no activity was seen for longer than timeoutMs.
   */
  bool inactivityExpired(uint32_t nowMs) const { return nowMs - lastActivityMs > timeoutMs; }
};
