# LAMP (LoRa Multi-Packet Protocol)

A lightweight reliable transport architecture for segmenting and transmitting large payloads over point-to-point and local LoRa networks.

## Architecture and Framing

**Chunk**:
A single transport-layer fragment carrying a portion of a segmented message along with a 9-byte Logical Header and CRC-16 suffix.
_Avoid_: Fragment, segment, sub-packet

**Physical Frame**:
A single over-the-air packet transmitted or received by the LoRa transceiver, subject to the transceiver FIFO and maximum LoRa payload limit (255 bytes).
_Avoid_: Packet, raw packet

**Logical Header**:
The 9-byte transport-layer header prepended to every LAMP chunk, containing addressing, sequence metadata, and control flags.
_Avoid_: Packet header, transport header

**CRC-16**:
A 2-byte cyclic redundancy check (Modbus polynomial 0xA001) appended to each chunk to detect frame-level corruption.
_Avoid_: Checksum, hash

## Session and State Management

**Reassembly Session**:
A receiver-side state machine descriptor that tracks and reassembles incoming chunks of a multi-chunk message using a bitfield.
_Avoid_: Context, receive buffer, incoming transfer

**Stateful Connection**:
A negotiated, peer-bound point-to-point association established via a 3-way handshake prior to data exchange.
_Avoid_: Link, session, socket

**Handshake (3WHS)**:
A three-way control frame exchange (SYN, SYN-ACK, ACK) used to establish a stateful connection and negotiate runtime session parameters.
_Avoid_: Negotiation, setup

**SACK-ARQ**:
A selective acknowledgment feedback mechanism where the receiver issues cumulative bitmap feedback upon sequence boundaries or explicit request.
_Avoid_: ACK/NACK, sliding window, selective repeat

## Operational Modes

**Mode 1 (Connectionless Best-Effort)**:
Unacknowledged, fire-and-forget transmission of segmented chunks without connection establishment.
_Avoid_: Unreliable mode, raw mode

**Mode 2 (Stateful Best-Effort)**:
Handshake-negotiated transmission without acknowledgment or chunk retransmission.
_Avoid_: Connected unacknowledged

**Mode 3 (Connectionless SACK-ARQ)**:
Autonomous segmented transmission with bitmap-based selective acknowledgments without prior connection setup.
_Avoid_: Standalone SACK, peer-to-peer reliable

**Mode 4 (Stateful SACK-ARQ)**:
Connection-oriented segmented transmission combining dynamic 3-way handshake parameter negotiation with bitmap-based selective retransmissions.
_Avoid_: Fully reliable mode, TCP mode

## Resource and Concurrency Bounds

**Session Admission Control**:
The mechanism that rejects incoming connection requests or drops unannounced connectionless bursts when free heap falls below safety margins or session tables saturate.
_Avoid_: Flow control, throttling
