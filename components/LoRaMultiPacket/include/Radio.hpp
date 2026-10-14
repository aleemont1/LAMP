#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @brief The part of the transceiver and of the platform that the protocol core uses.
 *
 * The core talks to these two interfaces instead of the concrete RadioLib classes, so that it builds and runs on a
 * host (unit tests drive it with a scripted radio and a manual clock) and the dependency on RadioLib stays in one
 * adapter (RadioLibAdapters.hpp).
 */
namespace RadioStatus
{
constexpr int OK = 0;
constexpr int FAILED = -1;
}  // namespace RadioStatus

namespace RadioIrq
{
constexpr uint32_t RX_DONE = 0x0002;  ///< value of RADIOLIB_SX126X_IRQ_RX_DONE
constexpr uint32_t ALL = 0x43FF;      ///< value of RADIOLIB_SX126X_IRQ_ALL
}  // namespace RadioIrq

class IRadio
{
 public:
  virtual ~IRadio() = default;

  virtual int transmit(const uint8_t *data, size_t len) = 0;
  virtual int startReceive() = 0;
  virtual int readData(uint8_t *data, size_t len) = 0;
  virtual size_t getPacketLength() = 0;
  virtual uint32_t getIrqFlags() = 0;
  virtual void clearIrqFlags(uint32_t flags) = 0;
  virtual uint64_t getTimeOnAirUs(size_t len) = 0;
  virtual float getRSSI() = 0;
  virtual float getSNR() = 0;
  virtual uint8_t randomByte() = 0;
};

class IPlatform
{
 public:
  virtual ~IPlatform() = default;

  /// Milliseconds since boot, from one monotonic source: every timer of the core is measured with it.
  virtual uint32_t millis() = 0;
  virtual void delay(uint32_t ms) = 0;
  /// Level of the radio's interrupt line (DIO1).
  virtual bool readIrqPin(uint32_t pin) = 0;
};
