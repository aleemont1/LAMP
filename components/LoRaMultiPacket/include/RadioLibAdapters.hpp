#pragma once

#include <RadioLib.h>

#include "Radio.hpp"

static_assert(RadioIrq::RX_DONE == RADIOLIB_SX126X_IRQ_RX_DONE, "IRQ bit layout changed");
static_assert(RadioIrq::ALL == RADIOLIB_SX126X_IRQ_ALL, "IRQ bit layout changed");
static_assert(RADIOLIB_ERR_NONE == RadioStatus::OK, "RadioLib status codes changed");

/// IRadio over an SX1262 driver.
class Sx1262Radio : public IRadio
{
 public:
  explicit Sx1262Radio(SX1262 *radio) : radio_(radio) {}

  int transmit(const uint8_t *data, size_t len) override { return radio_->transmit(data, len); }
  int startReceive() override { return radio_->startReceive(); }
  int readData(uint8_t *data, size_t len) override { return radio_->readData(data, len); }
  size_t getPacketLength() override { return radio_->getPacketLength(); }
  uint32_t getIrqFlags() override { return radio_->getIrqFlags(); }
  void clearIrqFlags(uint32_t flags) override { radio_->clearIrqFlags(flags); }
  uint64_t getTimeOnAirUs(size_t len) override { return radio_->getTimeOnAir(len); }
  float getRSSI() override { return radio_->getRSSI(); }
  float getSNR() override { return radio_->getSNR(); }
  uint8_t randomByte() override { return radio_->randomByte(); }

 private:
  SX1262 *radio_;
};

/// IPlatform over a RadioLib HAL.
class RadioLibPlatform : public IPlatform
{
 public:
  explicit RadioLibPlatform(RadioLibHal *hal) : hal_(hal) {}

  uint32_t millis() override { return static_cast<uint32_t>(hal_->millis()); }
  void delay(uint32_t ms) override { hal_->delay(ms); }
  bool readIrqPin(uint32_t pin) override { return hal_->digitalRead(pin) != 0; }

 private:
  RadioLibHal *hal_;
};
