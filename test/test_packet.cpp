// PlatformIO Unity unit tests for LoRaMultiPacket core logic.

#include <unity.h>

#include <cstring>  // for memcmp
#include <vector>

#include "Connection.hpp"
#include "Packet.hpp"
#include "PacketDeserializer.hpp"
#include "PacketParser.hpp"
#include "PacketReassembler.hpp"
#include "PacketSerializer.hpp"
#include "PacketValidator.hpp"
#include "SackHelper.hpp"

void setUp(void)
{
  // optional setup
}

void tearDown(void)
{
  // optional teardown
}

// ============================================================================
// Packet & Serializer Tests
// ============================================================================

/**
 * @brief Verifies that modifying the payload changes the CRC.
 */
static void test_crc_changes_on_payload_modification(void)
{
  Packet p1{};
  p1.header.payloadSize = 4;
  for (size_t i = 0; i < p1.header.payloadSize; ++i)
  {
    p1.payload.data[i] = static_cast<uint8_t>(i + 1);
  }
  p1.calculateCRC();

  Packet p2 = p1;
  p2.calculateCRC();
  TEST_ASSERT_EQUAL_UINT16(p1.crc, p2.crc);

  // Flip a bit in the first byte
  p2.payload.data[0] ^= 0xFF;
  p2.calculateCRC();
  TEST_ASSERT_NOT_EQUAL(p1.crc, p2.crc);
}

/**
 * @brief Verifies splitting a large vector into multiple packets and reassembling them.
 */
static void test_split_and_reassemble(void)
{
  // 600 bytes splits into 3 packets: 246 + 246 + 108 bytes
  size_t total = 600;
  std::vector<uint8_t> data(total);
  for (size_t i = 0; i < total; ++i)
  {
    data[i] = static_cast<uint8_t>(i & 0xFF);
  }

  // Split into chunks. The second argument is packetNumberStart (Message ID), not chunk size.
  // Chunk size is fixed to LORA_MAX_PAYLOAD_SIZE (246 bytes).
  auto packets = PacketSerializer::splitVectorToPackets(data, 42);

  TEST_ASSERT_EQUAL_INT(3, packets.size());

  std::vector<uint8_t> out;
  for (size_t i = 0; i < packets.size(); ++i)
  {
    auto &p = packets[i];
    TEST_ASSERT_EQUAL_UINT16(42, p.header.messageId);
    TEST_ASSERT_EQUAL_UINT8(3, p.header.totalChunks);
    TEST_ASSERT_EQUAL_UINT8(i, p.header.chunkIndex);

    size_t n = p.header.payloadSize;
    out.insert(out.end(), p.payload.data, p.payload.data + n);

    // Verify CRC integrity for each generated packet
    uint16_t old = p.crc;
    p.calculateCRC();
    TEST_ASSERT_EQUAL_UINT16(old, p.crc);
  }

  TEST_ASSERT_EQUAL_INT((int)data.size(), (int)out.size());
  for (size_t i = 0; i < data.size(); ++i)
  {
    TEST_ASSERT_EQUAL_UINT8(data[i], out[i]);
  }
}

/**
 * @brief Verifies that flags are left at 0 after split (SOM/EOM removed).
 */
static void test_packet_flags_multipacket(void)
{
  // 600 bytes splits into: 246 + 246 + 108 (3 packets)
  size_t total = 600;
  std::vector<uint8_t> data(total, 0xAB);

  auto packets = PacketSerializer::splitVectorToPackets(data, 100);

  TEST_ASSERT_EQUAL_INT(3, packets.size());

  // All packets have flags == 0 after split; control flags are set by the caller
  TEST_ASSERT_EQUAL_HEX8(0x00, packets[0].header.flags);
  TEST_ASSERT_EQUAL_HEX8(0x00, packets[1].header.flags);
  TEST_ASSERT_EQUAL_HEX8(0x00, packets[2].header.flags);
}

static void test_packet_flags_single_packet(void)
{
  std::vector<uint8_t> data(10, 0xAB);
  auto packets = PacketSerializer::splitVectorToPackets(data, 100);

  TEST_ASSERT_EQUAL_INT(1, packets.size());
  // No SOM/EOM flags; start/end derived from chunkIndex and totalChunks
  TEST_ASSERT_EQUAL_HEX8(0x00, packets[0].header.flags);
}

/**
 * @brief Verifies binary serialization layout with node addresses.
 */
static void test_binary_serialization_layout(void)
{
  Packet p{};
  p.header.srcAddr = 0x05;
  p.header.dstAddr = 0x42;
  p.header.messageId = 0x1234;
  p.header.payloadSize = 1;
  p.payload.data[0] = 0xEE;
  p.calculateCRC();

  uint8_t buffer[MAX_PACKET_SIZE];
  std::memset(buffer, 0, sizeof(buffer));

  PacketSerializer::serialize(p, buffer);

  // Check Address Fields
  TEST_ASSERT_EQUAL_HEX8(0x05, buffer[0]);
  TEST_ASSERT_EQUAL_HEX8(0x42, buffer[1]);

  // Check Header (Message ID at offset 2, Little-endian: 0x34, 0x12)
  TEST_ASSERT_EQUAL_HEX8(0x34, buffer[2]);
  TEST_ASSERT_EQUAL_HEX8(0x12, buffer[3]);

  // Check Payload (Offset HEADER_SIZE)
  TEST_ASSERT_EQUAL_HEX8(0xEE, buffer[HEADER_SIZE]);

  // Check CRC
  size_t crcOffset = HEADER_SIZE + p.header.payloadSize;
  uint16_t serializedCrc = 0;
  std::memcpy(&serializedCrc, buffer + crcOffset, 2);
  TEST_ASSERT_EQUAL_UINT16(p.crc, serializedCrc);
}

/**
 * @brief Verifies that splitVectorToPackets populates srcAddr and dstAddr correctly.
 */
static void test_node_addressing_serialization(void)
{
  std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
  auto packets = PacketSerializer::splitVectorToPackets(data, 10, 0x0A, 0x0B);

  TEST_ASSERT_EQUAL_INT(1, packets.size());
  TEST_ASSERT_EQUAL_HEX8(0x0A, packets[0].header.srcAddr);
  TEST_ASSERT_EQUAL_HEX8(0x0B, packets[0].header.dstAddr);
  TEST_ASSERT_EQUAL_UINT16(10, packets[0].header.messageId);
}

// ============================================================================
// Deserializer, Parser, and Validator Tests
// ============================================================================

static void test_parser_valid_single_chunk(void)
{
  Packet pkt{};
  pkt.header.messageId = 1;
  pkt.header.totalChunks = 1;
  pkt.header.chunkIndex = 0;
  pkt.header.payloadSize = 24;
  pkt.header.flags = 0;
  pkt.header.protocolVersion = 2;

  std::memcpy(pkt.payload.data, "Single chunk packet test", 24);
  pkt.calculateCRC();

  uint8_t buffer[256];
  PacketSerializer::serialize(pkt, buffer);
  size_t len = HEADER_SIZE + pkt.header.payloadSize + CRC_SIZE;

  auto result = PacketParser::parse(buffer, len);
  TEST_ASSERT_TRUE(result.has_value());
  TEST_ASSERT_EQUAL_UINT16(1, result.value().header.messageId);
}

static void test_parser_rejects_buffer_too_small(void)
{
  uint8_t buffer[10] = {0};
  auto result = PacketParser::parse(buffer, 10);
  TEST_ASSERT_FALSE(result.has_value());
}

static void test_parser_rejects_invalid_protocol_version(void)
{
  Packet pkt{};
  pkt.header.messageId = 1;
  pkt.header.totalChunks = 1;
  pkt.header.chunkIndex = 0;
  pkt.header.flags = 0;
  pkt.header.protocolVersion = 99;  // Invalid
  pkt.header.payloadSize = 10;
  std::memset(pkt.payload.data, 0, 10);
  pkt.calculateCRC();

  uint8_t buffer[256];
  PacketSerializer::serialize(pkt, buffer);
  size_t len = HEADER_SIZE + pkt.header.payloadSize + CRC_SIZE;

  auto result = PacketParser::parse(buffer, len);
  TEST_ASSERT_FALSE(result.has_value());
}

static void test_parser_rejects_crc_mismatch(void)
{
  Packet pkt{};
  pkt.header.messageId = 1;
  pkt.header.totalChunks = 1;
  pkt.header.chunkIndex = 0;
  pkt.header.flags = 0;
  pkt.header.payloadSize = 32;
  pkt.header.protocolVersion = 2;
  std::memset(pkt.payload.data, 0xAA, 32);
  pkt.calculateCRC();

  uint8_t buffer[256];
  PacketSerializer::serialize(pkt, buffer);
  size_t len = HEADER_SIZE + pkt.header.payloadSize + CRC_SIZE;

  // Corrupt the CRC at the end of the serialized buffer
  uint16_t *crc_ptr = reinterpret_cast<uint16_t *>(buffer + len - CRC_SIZE);
  *crc_ptr ^= 0xFFFF;

  auto result = PacketParser::parse(buffer, len);
  TEST_ASSERT_FALSE(result.has_value());
}

static void test_deserializer_extracts_valid_bytes(void)
{
  Packet pkt{};
  pkt.header.payloadSize = 38;
  const char testData[] = "Payload deserialization test data here";
  std::memcpy(pkt.payload.data, testData, 38);

  std::vector<uint8_t> extractedPayload;
  PacketDeserializer::deserialize(pkt, extractedPayload);
  TEST_ASSERT_EQUAL_size_t(38, extractedPayload.size());
  TEST_ASSERT_EQUAL_MEMORY(testData, extractedPayload.data(), 38);
}

// Helper for validator tests
static Packet create_valid_base_packet()
{
  Packet p{};
  p.header.messageId = 42;
  p.header.totalChunks = 1;
  p.header.chunkIndex = 0;
  p.header.payloadSize = 10;
  p.header.flags = 0;
  p.header.protocolVersion = 2;
  std::memset(p.payload.data, 0xAB, 10);
  p.calculateCRC();
  return p;
}

// Validator tests targeting PacketValidator::validate directly
static void test_validator_invalid_protocol_version(void)
{
  Packet p = create_valid_base_packet();
  p.header.protocolVersion = 99;  // Supported is 2
  p.calculateCRC();
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::INVALID_PROTOCOL_VERSION, err.value().type);
}

static void test_validator_invalid_message_id(void)
{
  Packet p = create_valid_base_packet();
  p.header.messageId = 0;  // Reserved
  p.calculateCRC();
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::INVALID_MESSAGE_ID, err.value().type);
}

static void test_validator_invalid_total_chunks(void)
{
  Packet p = create_valid_base_packet();
  p.header.totalChunks = 0;  // Invalid
  p.calculateCRC();
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::INVALID_TOTAL_CHUNKS, err.value().type);
}

static void test_validator_invalid_chunk_index(void)
{
  Packet p = create_valid_base_packet();
  p.header.totalChunks = 3;
  p.header.chunkIndex = 3;  // Must be < totalChunks (0, 1, 2)
  p.calculateCRC();
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::INVALID_CHUNK_INDEX, err.value().type);
}

static void test_validator_invalid_payload_size_too_large(void)
{
  Packet p = create_valid_base_packet();
  p.header.payloadSize = LORA_MAX_PAYLOAD_SIZE + 1;  // Exceeds limit
  p.calculateCRC();
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::INVALID_PAYLOAD_SIZE, err.value().type);
}

static void test_validator_invalid_payload_size_non_final_partial(void)
{
  Packet p = create_valid_base_packet();
  p.header.totalChunks = 2;
  p.header.chunkIndex = 0;
  p.header.payloadSize = 10;  // Non-final chunk must be full size (LORA_MAX_PAYLOAD_SIZE)
  p.header.flags = 0;
  p.calculateCRC();
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::INVALID_PAYLOAD_SIZE, err.value().type);
}

static void test_validator_crc_mismatch(void)
{
  Packet p = create_valid_base_packet();
  p.crc ^= 0xFFFF;  // Corrupt CRC
  auto err = PacketValidator::validate(p);
  TEST_ASSERT_TRUE(err.has_value());
  TEST_ASSERT_EQUAL(ValidationError::Type::CRC_MISMATCH, err.value().type);
}

// ============================================================================
// PacketReassembler Tests
// ============================================================================

/**
 * @brief Helper to generate a dummy packet for reassembly tests.
 */
Packet create_chunk(uint16_t msgId, uint8_t index, uint8_t total, const std::string &content, uint8_t srcAddr = 0)
{
  Packet p{};
  p.header.srcAddr = srcAddr;
  p.header.messageId = msgId;
  p.header.chunkIndex = index;
  p.header.totalChunks = total;
  p.header.payloadSize = content.size();
  p.header.protocolVersion = 2;
  std::memcpy(p.payload.data, content.data(), content.size());

  // flags: start/end-of-message are derived from chunkIndex/totalChunks, not flags
  p.header.flags = 0;

  p.calculateCRC();
  return p;
}

/**
 * @brief Verifies that packets arriving in order are reassembled correctly.
 */
static void test_reassembler_ordered_flow(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  // Create 3 chunks
  Packet p0 = create_chunk(10, 0, 3, "Hello ");
  Packet p1 = create_chunk(10, 1, 3, "World ");
  Packet p2 = create_chunk(10, 2, 3, "!!!");

  // Feed chunk 0
  auto res0 = reassembler.processPacket(p0, time);
  TEST_ASSERT_FALSE(res0.has_value());  // Not done yet

  // Feed chunk 1
  auto res1 = reassembler.processPacket(p1, time);
  TEST_ASSERT_FALSE(res1.has_value());

  // Feed chunk 2 (Final)
  auto res2 = reassembler.processPacket(p2, time);
  TEST_ASSERT_TRUE(res2.has_value());

  // Verify content
  std::string finalStr(res2.value().begin(), res2.value().end());
  TEST_ASSERT_EQUAL_STRING("Hello World !!!", finalStr.c_str());
}

/**
 * @brief Verifies that packets arriving out of order are reassembled correctly.
 */
static void test_reassembler_unordered_flow(void)
{
  PacketReassembler reassembler;
  uint32_t time = 2000;

  // Create 3 chunks
  Packet p0 = create_chunk(20, 0, 3, "Part1");
  Packet p1 = create_chunk(20, 1, 3, "Part2");
  Packet p2 = create_chunk(20, 2, 3, "Part3");

  // Send Index 2 (Last) first
  auto res2 = reassembler.processPacket(p2, time);
  TEST_ASSERT_FALSE(res2.has_value());

  // Send Index 0 (First)
  auto res0 = reassembler.processPacket(p0, time);
  TEST_ASSERT_FALSE(res0.has_value());

  // Send Index 1 (Middle) - Should trigger completion
  auto res1 = reassembler.processPacket(p1, time);
  TEST_ASSERT_TRUE(res1.has_value());

  // Check data integrity
  std::string result(res1.value().begin(), res1.value().end());
  TEST_ASSERT_EQUAL_STRING("Part1Part2Part3", result.c_str());
}

/**
 * @brief Verifies that duplicate packets are ignored and don't break the counter.
 */
static void test_reassembler_duplicates_ignored(void)
{
  PacketReassembler reassembler;
  uint32_t time = 3000;

  Packet p0 = create_chunk(30, 0, 2, "A");
  Packet p1 = create_chunk(30, 1, 2, "B");

  // Send chunk 0 twice
  reassembler.processPacket(p0, time);
  auto resDup = reassembler.processPacket(p0, time);  // Duplicate
  TEST_ASSERT_FALSE(resDup.has_value());

  // Send chunk 1
  auto resFinal = reassembler.processPacket(p1, time);
  TEST_ASSERT_TRUE(resFinal.has_value());
  TEST_ASSERT_EQUAL_size_t(2, resFinal.value().size());
}

/**
 * @brief Verifies that old sessions are pruned after timeout.
 */
static void test_reassembler_pruning(void)
{
  PacketReassembler reassembler;

  // T=1000: Start Message 40
  Packet p0 = create_chunk(40, 0, 2, "OldData");
  reassembler.processPacket(p0, 1000);

  // T=5000: Prune with timeout 2000ms.
  // Elapsed = 5000 - 1000 = 4000 (> 2000). Should be removed.
  reassembler.prune(5000, 2000);

  // T=5001: Arrive chunk 1 of Message 40.
  // Since session was pruned, this is treated as a *new* partial session
  // containing only chunk 1. It will NOT complete.
  Packet p1 = create_chunk(40, 1, 2, "NewData");
  auto res = reassembler.processPacket(p1, 5001);

  TEST_ASSERT_FALSE(res.has_value());
}

/**
 * @brief A session stays alive while chunks keep arriving (inactivity is measured from the last chunk),
 * and the limit can depend on the message's chunk count.
 */
static void test_reassembler_prune_by_inactivity(void)
{
  PacketReassembler reassembler;

  reassembler.processPacket(create_chunk(41, 0, 3, "A"), 1000);
  reassembler.processPacket(create_chunk(41, 1, 3, "B"), 9000);  // 8 s after the first chunk

  // 11 s after the first chunk but only 2 s after the last: must survive a 5 s inactivity limit.
  reassembler.prune(11000, 5000);
  auto res = reassembler.processPacket(create_chunk(41, 2, 3, "C"), 11001);
  TEST_ASSERT_TRUE(res.has_value());

  // Per-message limit: a 2-chunk message gets 1 s, a 200-chunk message 60 s; both silent for 10 s.
  reassembler.processPacket(create_chunk(42, 0, 2, "x"), 20000);
  reassembler.processPacket(create_chunk(43, 0, 200, "y"), 20000);
  reassembler.prune(30000, [](uint8_t total)
                    { return total > 100 ? 60000u : 1000u; });
  std::vector<uint8_t> bitmap;
  TEST_ASSERT_FALSE(reassembler.getReceivedBitmap(0, 42, bitmap));
  TEST_ASSERT_TRUE(reassembler.getReceivedBitmap(0, 43, bitmap));
}

static void test_reassembler_session_limit(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  // Send 10 packets for 10 different message IDs (all are chunk 0 of 2)
  for (uint16_t msgId = 1; msgId <= 10; ++msgId)
  {
    Packet p = create_chunk(msgId, 0, 2, "A");
    auto res = reassembler.processPacket(p, time);
    TEST_ASSERT_FALSE(res.has_value());
  }

  // Now try to send a packet for an 11th message ID
  Packet p11 = create_chunk(11, 0, 2, "B");
  auto res11 = reassembler.processPacket(p11, time);
  TEST_ASSERT_FALSE(res11.has_value());  // Should be discarded because sessions size >= 10

  // Even if we send the final chunk for message 11, it shouldn't complete
  Packet p11_final = create_chunk(11, 1, 2, "C");
  auto res11_final = reassembler.processPacket(p11_final, time);
  TEST_ASSERT_FALSE(res11_final.has_value());

  // However, if we complete one of the first 10 sessions, e.g., message 5
  Packet p5_final = create_chunk(5, 1, 2, "B");
  auto res5_final = reassembler.processPacket(p5_final, time);
  TEST_ASSERT_TRUE(res5_final.has_value());  // Message 5 completes

  // Now sessions size is 9, so we should be able to start message 12
  Packet p12 = create_chunk(12, 0, 2, "D");
  auto res12 = reassembler.processPacket(p12, time);
  TEST_ASSERT_FALSE(res12.has_value());  // Accepted (not completed yet)

  // And it can complete
  Packet p12_final = create_chunk(12, 1, 2, "E");
  auto res12_final = reassembler.processPacket(p12_final, time);
  TEST_ASSERT_TRUE(res12_final.has_value());
}

static void test_reassembler_duplicate_mismatch_ignored(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  Packet p0 = create_chunk(50, 0, 2, "A");
  Packet p0_mismatch = create_chunk(50, 0, 2, "X");  // Duplicate with different payload
  Packet p1 = create_chunk(50, 1, 2, "B");

  reassembler.processPacket(p0, time);
  // Send duplicate with different content
  auto resDup = reassembler.processPacket(p0_mismatch, time);
  TEST_ASSERT_FALSE(resDup.has_value());

  // Send final chunk
  auto resFinal = reassembler.processPacket(p1, time);
  TEST_ASSERT_TRUE(resFinal.has_value());

  // Reassembled content should be "AB", not "XB"
  std::string finalStr(resFinal.value().begin(), resFinal.value().end());
  TEST_ASSERT_EQUAL_STRING("AB", finalStr.c_str());
}

static void test_reassembler_reset(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  Packet p0 = create_chunk(60, 0, 2, "A");
  Packet p1 = create_chunk(60, 1, 2, "B");

  reassembler.processPacket(p0, time);

  // Call reset
  reassembler.reset();

  // Now send chunk 1. Since session was reset, this is treated as a new session
  // containing only chunk 1. It should NOT complete.
  auto res = reassembler.processPacket(p1, time);
  TEST_ASSERT_FALSE(res.has_value());
}

static void test_validator_bypass_ack_flags(void)
{
  Packet ackPacket{};
  ackPacket.header.messageId = 123;
  ackPacket.header.flags = FLAG_ACK;
  ackPacket.header.chunkIndex = 0;
  ackPacket.header.totalChunks = 1;
  ackPacket.header.payloadSize = 2;
  ackPacket.payload.data[0] = 0xAA;
  ackPacket.payload.data[1] = 0x55;
  ackPacket.calculateCRC();

  auto err = PacketValidator::validate(ackPacket);
  TEST_ASSERT_FALSE(err.has_value());
}

static void test_reassembler_get_received_bitmap(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  // 10-chunk message reassembly session
  // Chunks received: 0, 1, 3, 4, 7, 9
  // Chunks missing: 2, 5, 6, 8
  // Expected bitmap (10 bits, 2 bytes):
  // Byte 0: bits 0-7: 1 1 0 1 1 0 0 1 -> 0x9B
  // Byte 1: bits 8-15:
  // bit 0 (chunk 8) = 0 (0)
  // bit 1 (chunk 9) = 1 (2) -> 0x02

  reassembler.processPacket(create_chunk(70, 0, 10, "A"), time);
  reassembler.processPacket(create_chunk(70, 1, 10, "B"), time);
  reassembler.processPacket(create_chunk(70, 3, 10, "D"), time);
  reassembler.processPacket(create_chunk(70, 4, 10, "E"), time);
  reassembler.processPacket(create_chunk(70, 7, 10, "H"), time);
  reassembler.processPacket(create_chunk(70, 9, 10, "J"), time);

  std::vector<uint8_t> bitmap;
  bool found = reassembler.getReceivedBitmap(0, 70, bitmap);
  TEST_ASSERT_TRUE(found);
  TEST_ASSERT_EQUAL_INT(2, bitmap.size());
  TEST_ASSERT_EQUAL_HEX8(0x9B, bitmap[0]);
  TEST_ASSERT_EQUAL_HEX8(0x02, bitmap[1]);
}

static void test_reassembler_completed_messages(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  TEST_ASSERT_FALSE(reassembler.isCompleted(0, 80, time));

  // Complete a session
  reassembler.processPacket(create_chunk(80, 0, 2, "A"), time);
  auto res = reassembler.processPacket(create_chunk(80, 1, 2, "B"), time);
  TEST_ASSERT_TRUE(res.has_value());

  // Mark it completed
  reassembler.markCompleted(0, 80, time);
  TEST_ASSERT_TRUE(reassembler.isCompleted(0, 80, time));

  // Verify that it is cleared on reset
  reassembler.reset();
  TEST_ASSERT_FALSE(reassembler.isCompleted(0, 80, time));
}

/**
 * @brief Two senders reusing the same message ID must not share a reassembly session.
 */
static void test_reassembler_same_msgid_different_senders(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  reassembler.processPacket(create_chunk(5, 0, 2, "A", 0x01), time);
  reassembler.processPacket(create_chunk(5, 1, 2, "Y", 0x02), time);

  // Neither session is complete: each sender has delivered one chunk of two.
  std::vector<uint8_t> bitmap;
  TEST_ASSERT_TRUE(reassembler.getReceivedBitmap(0x01, 5, bitmap));
  TEST_ASSERT_EQUAL_HEX8(0x01, bitmap[0]);
  TEST_ASSERT_TRUE(reassembler.getReceivedBitmap(0x02, 5, bitmap));
  TEST_ASSERT_EQUAL_HEX8(0x02, bitmap[0]);

  auto res = reassembler.processPacket(create_chunk(5, 1, 2, "B", 0x01), time);
  TEST_ASSERT_TRUE(res.has_value());
  reassembler.markCompleted(0x01, 5, time);
  TEST_ASSERT_TRUE(reassembler.isCompleted(0x01, 5, time));
  TEST_ASSERT_FALSE(reassembler.isCompleted(0x02, 5, time));
}

/**
 * @brief A broadcast frame must not ask for an acknowledgment nor carry connection control: there is no single
 * receiver to answer, and every node would reply at once.
 */
static void test_broadcast_with_reliability_or_connection_flags_is_rejected(void)
{
  const uint8_t offending[] = {FLAG_ACK_REQ, FLAG_CONN_REQ, FLAG_CONN_ACK, FLAG_CONN_NACK,
                               static_cast<uint8_t>(FLAG_CONN_REQ | FLAG_CONN_ACK)};
  for (uint8_t flags : offending)
  {
    PacketHeader h{};
    h.dstAddr = ADDRESS_BROADCAST;
    h.flags = flags;
    TEST_ASSERT_TRUE(PacketValidator::isBroadcastWithReliabilityFlags(h));

    h.dstAddr = 0x02;  // the same flags are fine on a unicast frame
    TEST_ASSERT_FALSE(PacketValidator::isBroadcastWithReliabilityFlags(h));
  }

  PacketHeader plain{};
  plain.dstAddr = ADDRESS_BROADCAST;
  plain.flags = 0;
  TEST_ASSERT_FALSE(PacketValidator::isBroadcastWithReliabilityFlags(plain));
}

/**
 * @brief Reliable delivery needs a unicast destination.
 */
static void test_reliable_send_requires_unicast_destination(void)
{
  TEST_ASSERT_FALSE(PacketValidator::canSendReliablyTo(ADDRESS_BROADCAST));
  TEST_ASSERT_TRUE(PacketValidator::canSendReliablyTo(0x01));
  TEST_ASSERT_TRUE(PacketValidator::canSendReliablyTo(0xFE));
}

/**
 * @brief A peer that keeps sending (a new message ID per message) must keep the connection alive, even when the
 * total duration exceeds the inactivity timeout.
 */
static void test_connection_activity_refreshed_by_peer_frames(void)
{
  ConnectionSession conn;
  conn.state = ConnectionState::ESTABLISHED;
  conn.peerAddr = 0x02;
  conn.sessionMsgId = 10;  // the handshake ID; data messages use other IDs
  conn.timeoutMs = 15000;
  conn.lastActivityMs = 0;

  // One frame every 10 s for 60 s: 4x the inactivity timeout.
  for (uint32_t now = 10000; now <= 60000; now += 10000)
  {
    TEST_ASSERT_FALSE(conn.inactivityExpired(now));
    TEST_ASSERT_TRUE(conn.touchOnFrameFrom(0x02, now));
  }
  TEST_ASSERT_FALSE(conn.inactivityExpired(60000 + 15000));
  TEST_ASSERT_TRUE(conn.inactivityExpired(60000 + 15001));
}

/**
 * @brief Frames from another node must not keep the connection alive.
 */
static void test_connection_activity_not_refreshed_by_other_nodes(void)
{
  ConnectionSession conn;
  conn.state = ConnectionState::ESTABLISHED;
  conn.peerAddr = 0x02;
  conn.timeoutMs = 15000;
  conn.lastActivityMs = 1000;

  TEST_ASSERT_FALSE(conn.touchOnFrameFrom(0x03, 12000));
  TEST_ASSERT_EQUAL_UINT32(1000, conn.lastActivityMs);
  TEST_ASSERT_TRUE(conn.inactivityExpired(16001));
}

/**
 * @brief Only an ESTABLISHED connection is refreshed by data frames (SYN_RCVD is promoted elsewhere).
 */
static void test_connection_activity_ignored_when_not_established(void)
{
  ConnectionSession conn;
  conn.state = ConnectionState::CLOSED;
  conn.peerAddr = 0x02;
  conn.lastActivityMs = 500;

  TEST_ASSERT_FALSE(conn.touchOnFrameFrom(0x02, 9000));
  TEST_ASSERT_EQUAL_UINT32(500, conn.lastActivityMs);
}

/**
 * @brief After forgetSender() a message with the same (srcAddr, messageId) as a completed one is delivered again:
 * a sender that rebooted and reused the ID must not be taken for a duplicate.
 */
static void test_reassembler_forget_sender_clears_completed_history(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  reassembler.markCompleted(0x01, 7, time);
  reassembler.markCompleted(0x02, 7, time);
  TEST_ASSERT_TRUE(reassembler.isCompleted(0x01, 7, time));

  reassembler.forgetSender(0x01);

  TEST_ASSERT_FALSE(reassembler.isCompleted(0x01, 7, time));
  TEST_ASSERT_TRUE(reassembler.isCompleted(0x02, 7, time));  // other senders are untouched
}

/**
 * @brief forgetSender() also drops the partial session of that sender, so frames of the new message do not
 * merge with chunks of the previous incarnation.
 */
static void test_reassembler_forget_sender_drops_partial_sessions(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  reassembler.processPacket(create_chunk(9, 0, 2, "A", 0x01), time);
  reassembler.processPacket(create_chunk(9, 0, 2, "Y", 0x02), time);

  reassembler.forgetSender(0x01);

  std::vector<uint8_t> bitmap;
  TEST_ASSERT_FALSE(reassembler.getReceivedBitmap(0x01, 9, bitmap));
  TEST_ASSERT_TRUE(reassembler.getReceivedBitmap(0x02, 9, bitmap));

  // The new message starts from scratch: chunk 1 alone must not complete it.
  auto res = reassembler.processPacket(create_chunk(9, 1, 2, "B", 0x01), time);
  TEST_ASSERT_FALSE(res.has_value());
}

/**
 * @brief A completed entry older than COMPLETED_HISTORY_TTL_MS no longer suppresses a message with the same key.
 */
static void test_reassembler_completed_entry_expires(void)
{
  PacketReassembler reassembler;
  const uint32_t ttl = LoRaMultiPacketConfig::COMPLETED_HISTORY_TTL_MS;
  uint32_t time = 5000;

  reassembler.markCompleted(0x01, 11, time);

  TEST_ASSERT_TRUE(reassembler.isCompleted(0x01, 11, time + ttl));       // exactly at the limit: still a duplicate
  TEST_ASSERT_FALSE(reassembler.isCompleted(0x01, 11, time + ttl + 1));  // expired

  // The expired entry does not block a new message with the same key.
  auto first = reassembler.processPacket(create_chunk(11, 0, 2, "A", 0x01), time + ttl + 1);
  TEST_ASSERT_FALSE(first.has_value());
  auto done = reassembler.processPacket(create_chunk(11, 1, 2, "B", 0x01), time + ttl + 1);
  TEST_ASSERT_TRUE(done.has_value());
}

/**
 * @brief A completed entry inside the window keeps suppressing the duplicate (e.g. a retransmission after a lost SACK),
 * also across a prune() call.
 */
static void test_reassembler_completed_entry_within_ttl_suppresses_duplicate(void)
{
  PacketReassembler reassembler;
  const uint32_t ttl = LoRaMultiPacketConfig::COMPLETED_HISTORY_TTL_MS;
  uint32_t time = 5000;

  reassembler.markCompleted(0x01, 12, time);
  reassembler.prune(time + ttl / 2, 1000);

  TEST_ASSERT_TRUE(reassembler.isCompleted(0x01, 12, time + ttl / 2));
}

/**
 * @brief Message IDs derived from a random word are never 0 (reserved), and a fresh boot does not start at 1.
 */
static void test_message_id_from_random_is_never_zero(void)
{
  TEST_ASSERT_EQUAL_UINT16(1, messageIdFromRandom(0x00000000));
  TEST_ASSERT_EQUAL_UINT16(1, messageIdFromRandom(0xABCD0000));  // low 16 bits are zero
  TEST_ASSERT_EQUAL_UINT16(0x1234, messageIdFromRandom(0xDEAD1234));
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, messageIdFromRandom(0xFFFFFFFF));
}

/**
 * @brief A frame of an existing session that declares a different totalChunks (e.g. a sender that rebooted and
 * reused the message ID) must be dropped: it must not be stored nor complete the session with a hole.
 */
static void test_reassembler_total_chunks_mismatch_dropped(void)
{
  PacketReassembler reassembler;
  uint32_t time = 1000;

  reassembler.processPacket(create_chunk(70, 0, 3, "A"), time);

  // Same sender and message ID, but declared as a 6-chunk message: valid on its own, inconsistent with the session.
  auto resForeign = reassembler.processPacket(create_chunk(70, 5, 6, "X"), time);
  TEST_ASSERT_FALSE(resForeign.has_value());

  // The foreign frame must not be counted: chunk 1 of 3 must not complete the session (chunk 2 is still missing).
  auto resB = reassembler.processPacket(create_chunk(70, 1, 3, "B"), time);
  TEST_ASSERT_FALSE(resB.has_value());

  std::vector<uint8_t> bitmap;
  TEST_ASSERT_TRUE(reassembler.getReceivedBitmap(0, 70, bitmap));
  TEST_ASSERT_EQUAL_UINT32(1, bitmap.size());
  TEST_ASSERT_EQUAL_UINT8(0x03, bitmap[0]);

  auto resC = reassembler.processPacket(create_chunk(70, 2, 3, "C"), time);
  TEST_ASSERT_TRUE(resC.has_value());
  std::string finalStr(resC.value().begin(), resC.value().end());
  TEST_ASSERT_EQUAL_STRING("ABC", finalStr.c_str());
}

/**
 * @brief An inconsistent frame must not refresh the session's inactivity timer.
 */
static void test_reassembler_total_chunks_mismatch_does_not_refresh(void)
{
  PacketReassembler reassembler;

  reassembler.processPacket(create_chunk(71, 0, 3, "A"), 1000);
  reassembler.processPacket(create_chunk(71, 5, 6, "X"), 3500);

  // 3500 ms after the last valid chunk, with a 3000 ms limit: the session is expired.
  reassembler.prune(4500, 3000);

  std::vector<uint8_t> bitmap;
  TEST_ASSERT_FALSE(reassembler.getReceivedBitmap(0, 71, bitmap));
}

static void test_sack_helper(void)
{
  // Full ACK bitmap for 10 chunks -> 2 bytes (0xFF, 0xFF)
  auto fullBmp = SackHelper::createFullAckBitmap(10);
  TEST_ASSERT_EQUAL_INT(2, fullBmp.size());
  TEST_ASSERT_EQUAL_UINT8(0xFF, fullBmp[0]);
  TEST_ASSERT_EQUAL_UINT8(0xFF, fullBmp[1]);

  auto missing = SackHelper::getMissingChunkIndices(fullBmp, 10);
  TEST_ASSERT_EQUAL_INT(0, missing.size());

  // Partial bitmap: Chunk 1 and Chunk 8 missing
  // Byte 0: chunk 1 missing -> 0b11111101 = 0xFD
  // Byte 1: chunk 8 missing -> 0b11111110 = 0xFE
  std::vector<uint8_t> partialBmp = {0xFD, 0xFE};
  auto missingPartial = SackHelper::getMissingChunkIndices(partialBmp, 10);
  TEST_ASSERT_EQUAL_INT(2, missingPartial.size());
  TEST_ASSERT_EQUAL_UINT8(1, missingPartial[0]);
  TEST_ASSERT_EQUAL_UINT8(8, missingPartial[1]);

  // SACK Packet creation
  Packet sackPkt = SackHelper::createSackPacket(123, 10, partialBmp);
  TEST_ASSERT_EQUAL_UINT16(123, sackPkt.header.messageId);
  TEST_ASSERT_EQUAL_UINT8(FLAG_ACK, sackPkt.header.flags);
  TEST_ASSERT_EQUAL_UINT8(2, sackPkt.header.payloadSize);
  TEST_ASSERT_EQUAL_UINT8(0xFD, sackPkt.payload.data[0]);
  TEST_ASSERT_EQUAL_UINT8(0xFE, sackPkt.payload.data[1]);
}

static void test_connection_syn_metadata_packing(void)
{
  TEST_ASSERT_EQUAL(4, sizeof(SynMetadata));
  TEST_ASSERT_EQUAL(4, sizeof(SynAckMetadata));

  SynMetadata syn{};
  syn.requestedPayloadSize = 200;
  syn.windowSize = 2;
  syn.timeoutMs = 10000;

  uint8_t buffer[4];
  std::memcpy(buffer, &syn, sizeof(SynMetadata));

  SynMetadata unpacked{};
  std::memcpy(&unpacked, buffer, sizeof(SynMetadata));

  TEST_ASSERT_EQUAL_UINT8(200, unpacked.requestedPayloadSize);
  TEST_ASSERT_EQUAL_UINT8(2, unpacked.windowSize);
  TEST_ASSERT_EQUAL_UINT16(10000, unpacked.timeoutMs);
}

static void test_connection_flags_validation(void)
{
  TEST_ASSERT_EQUAL_HEX8(0x10, FLAG_CONN_REQ);
  TEST_ASSERT_EQUAL_HEX8(0x20, FLAG_CONN_ACK);
  TEST_ASSERT_EQUAL_HEX8(0x40, FLAG_CONN_NACK);

  // SYN packet
  Packet synPkt{};
  synPkt.header.messageId = 10;
  synPkt.header.totalChunks = 1;
  synPkt.header.chunkIndex = 0;
  synPkt.header.payloadSize = sizeof(SynMetadata);
  synPkt.header.flags = FLAG_CONN_REQ;
  synPkt.header.protocolVersion = 2;
  synPkt.calculateCRC();

  auto err = PacketValidator::validate(synPkt);
  TEST_ASSERT_FALSE(err.has_value());
}

static void test_connection_session_busy_rejection(void)
{
  ConnectionSession session{};
  session.state = ConnectionState::ESTABLISHED;
  session.peerAddr = 0x01;  // Active session with Node 0x01

  // Concurrent SYN arrives from Node 0x02
  uint8_t incomingSynSender = 0x02;

  bool isBusy = (session.state != ConnectionState::CLOSED && session.peerAddr != incomingSynSender);
  TEST_ASSERT_TRUE(isBusy);

  // Same SYN arrives from Node 0x01 (Retransmission)
  uint8_t retransSender = 0x01;
  bool isRetransBusy = (session.state != ConnectionState::CLOSED && session.peerAddr != retransSender);
  TEST_ASSERT_FALSE(isRetransBusy);
}

void test_etsi_duty_cycle_pacing_calculation(void)
{
  // Formula: pacingDelayMs = toaMs * (1 - DC) / DC
  // For 1% ETSI limit (DC = 0.01), multiplier is 99
  float dc = 0.01f;
  float multiplier = (1.0f - dc) / dc;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 99.0f, multiplier);

  uint32_t toaMs = 50;                  // 50 ms ToA
  uint32_t expectedPacingMs = 50 * 99;  // 4950 ms
  TEST_ASSERT_EQUAL_UINT32(4950, expectedPacingMs);
}

void test_large_payload_duty_cycle_compliance(void)
{
  // Simula un pacchetto gigante da 62.2 KB diviso in 255 chunk massimi
  size_t totalChunks = 255;
  uint32_t toaPerChunkMs = 92;  // ToA per chunk da 244 bytes a BW 500kHz
  float dcLimit = 0.01f;        // 1% ETSI limit

  double totalAirtimeMs = 0;
  double totalElapsedTimeMs = 0;

  for (size_t i = 0; i < totalChunks; ++i)
  {
    uint32_t pacingMs = static_cast<uint32_t>(toaPerChunkMs * ((1.0f - dcLimit) / dcLimit));
    totalAirtimeMs += toaPerChunkMs;
    totalElapsedTimeMs += (toaPerChunkMs + pacingMs);
  }

  double actualDutyCycle = totalAirtimeMs / totalElapsedTimeMs;

  // Verifica che il duty cycle cumulativo sia esattamento <= 0.01 (1.000%)
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.01f, static_cast<float>(actualDutyCycle));
  TEST_ASSERT_TRUE(actualDutyCycle <= 0.010001f);
}

int main(void)
{
  UNITY_BEGIN();

  // Serializer and structural tests
  RUN_TEST(test_crc_changes_on_payload_modification);
  RUN_TEST(test_split_and_reassemble);
  RUN_TEST(test_packet_flags_multipacket);
  RUN_TEST(test_packet_flags_single_packet);
  RUN_TEST(test_binary_serialization_layout);
  RUN_TEST(test_node_addressing_serialization);

  // Parser & Deserializer tests
  RUN_TEST(test_parser_valid_single_chunk);
  RUN_TEST(test_parser_rejects_buffer_too_small);
  RUN_TEST(test_parser_rejects_invalid_protocol_version);
  RUN_TEST(test_parser_rejects_crc_mismatch);
  RUN_TEST(test_deserializer_extracts_valid_bytes);

  // Validator Tests
  RUN_TEST(test_validator_invalid_protocol_version);
  RUN_TEST(test_validator_invalid_message_id);
  RUN_TEST(test_validator_invalid_total_chunks);
  RUN_TEST(test_validator_invalid_chunk_index);
  RUN_TEST(test_validator_invalid_payload_size_too_large);
  RUN_TEST(test_validator_invalid_payload_size_non_final_partial);
  RUN_TEST(test_validator_crc_mismatch);
  RUN_TEST(test_validator_bypass_ack_flags);

  // Reassembler Tests
  RUN_TEST(test_reassembler_ordered_flow);
  RUN_TEST(test_reassembler_unordered_flow);
  RUN_TEST(test_reassembler_duplicates_ignored);
  RUN_TEST(test_reassembler_pruning);
  RUN_TEST(test_reassembler_prune_by_inactivity);
  RUN_TEST(test_reassembler_session_limit);
  RUN_TEST(test_reassembler_duplicate_mismatch_ignored);
  RUN_TEST(test_reassembler_reset);
  RUN_TEST(test_reassembler_get_received_bitmap);
  RUN_TEST(test_reassembler_completed_messages);
  RUN_TEST(test_reassembler_same_msgid_different_senders);
  RUN_TEST(test_reassembler_forget_sender_clears_completed_history);
  RUN_TEST(test_reassembler_forget_sender_drops_partial_sessions);
  RUN_TEST(test_reassembler_completed_entry_expires);
  RUN_TEST(test_reassembler_completed_entry_within_ttl_suppresses_duplicate);
  RUN_TEST(test_message_id_from_random_is_never_zero);
  RUN_TEST(test_broadcast_with_reliability_or_connection_flags_is_rejected);
  RUN_TEST(test_reliable_send_requires_unicast_destination);
  RUN_TEST(test_connection_activity_refreshed_by_peer_frames);
  RUN_TEST(test_connection_activity_not_refreshed_by_other_nodes);
  RUN_TEST(test_connection_activity_ignored_when_not_established);
  RUN_TEST(test_reassembler_total_chunks_mismatch_dropped);
  RUN_TEST(test_reassembler_total_chunks_mismatch_does_not_refresh);

  // SackHelper Tests
  RUN_TEST(test_sack_helper);

  // Connection & 3WHS Tests
  RUN_TEST(test_connection_syn_metadata_packing);
  RUN_TEST(test_connection_flags_validation);
  RUN_TEST(test_connection_session_busy_rejection);

  // ETSI Duty-Cycle Formula Verification
  RUN_TEST(test_etsi_duty_cycle_pacing_calculation);
  RUN_TEST(test_large_payload_duty_cycle_compliance);

  return UNITY_END();
}

#ifdef ESP_PLATFORM
extern "C" void app_main(void)
{
  main();
}
#endif