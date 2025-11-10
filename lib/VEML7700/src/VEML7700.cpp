/*** VEML7700 Light Sensor Library
 *  for Arduino and ESP32
 *  Copyright 2025 by yuttapichai - confidential
 *  DO NOT DISTRIBUTE without permission.
 *  DO NOT USE in commercial products without permission.
 *  DO NOT MODIFY without permission.
 *  DO NOT REMOVE this header.
 *  Created by yuttapichai.
 *  Contact: yuttapichai.leelai@gmail.com
 */

#include <Arduino.h>
#include "VEML7700.h"

// ---------- Ctor ----------
VEML7700::VEML7700(TwoWire &wire, uint8_t i2c_addr)
: wire_(&wire), addr_(i2c_addr) {}

// ---------- begin ----------
bool VEML7700::begin(const VEML7700_Config &cfg, int sda, int scl, uint32_t freq) {
  if (sda >= 0 && scl >= 0) {
    wire_->begin(sda, scl, freq);
  } else {
    // assume Wire already began
  }

  uint16_t id;
  readID(id); // optional

  cfg_ = cfg;
  if (!applyConfig(cfg_)) return false;

  // Disable PSM for simplicity
  if (!write16_(REG_PSM, 0x0000)) return false;

  return true;
}

// ---------- config ----------
bool VEML7700::applyConfig(const VEML7700_Config &cfg) {
  cfg_ = cfg;
  const uint16_t conf0 = packConf0_(cfg_);
  return write16_(REG_ALS_CONF_0, conf0);
}

bool VEML7700::setGain(VEML7700_Gain g) {
  cfg_.gain = g;
  return applyConfig(cfg_);
}

bool VEML7700::setIntegrationTime(VEML7700_IT it) {
  cfg_.it = it;
  return applyConfig(cfg_);
}

// ---------- raw ----------
bool VEML7700::readALS(uint16_t &rawALS)   { return read16_(REG_ALS_DATA, rawALS); }
bool VEML7700::readWHITE(uint16_t &rawW)   { return read16_(REG_WHITE_DATA, rawW); }
bool VEML7700::readID(uint16_t &id)        { return read16_(REG_ID, id); }

// ---------- lux ----------
bool VEML7700::readLux(float &lux) const {
  uint16_t raw = 0;
  if (!const_cast<VEML7700*>(this)->readALS(raw)) return false;
  const float res = luxPerCount_(cfg_.gain, cfg_.it);
  lux = static_cast<float>(raw) * res * cal_factor_;
  return true;
}

// ---------- I2C primitives ----------
bool VEML7700::write16_(uint8_t reg, uint16_t value) const {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write((uint8_t)(value & 0xFF));        // LSB first
  wire_->write((uint8_t)((value >> 8) & 0xFF)); // MSB
  return wire_->endTransmission(true) == 0;
}

bool VEML7700::read16_(uint8_t reg, uint16_t &out) const {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom((uint8_t)addr_, (uint8_t)2, (uint8_t)true) != 2) return false;
  const uint8_t lsb = wire_->read();
  const uint8_t msb = wire_->read();
  out = (uint16_t)(msb << 8) | lsb;
  return true;
}

// ---------- helpers ----------
uint16_t VEML7700::packConf0_(const VEML7700_Config &c) {
  uint16_t v = 0;
  v |= (uint16_t(c.gain) & 0b11)  << 11; // bits 12:11
  v |= (uint16_t(c.it)   & 0b1111)<< 6;  // bits 9:6
  v |= (uint16_t(c.pers) & 0b11)  << 4;  // bits 5:4
  if (c.int_en)   v |= (1 << 1);
  if (c.shutdown) v |= (1 << 0);
  return v;
}

uint16_t VEML7700::itMs_(VEML7700_IT it) {
  switch (it) {
    case IT_25ms:  return 25;
    case IT_50ms:  return 50;
    case IT_100ms: return 100;
    case IT_200ms: return 200;
    case IT_400ms: return 400;
    case IT_800ms: return 800;
    default:       return 100;
  }
}

float VEML7700::gainMul_(VEML7700_Gain g) {
  switch (g) {
    case GAIN_X2:   return 2.0f;
    case GAIN_X1:   return 1.0f;
    case GAIN_X1_4: return 0.25f;
    case GAIN_X1_8: return 0.125f;
    default:        return 2.0f;
  }
}

// Base: gain x2, IT 800 ms -> 0.0042 lx/count
float VEML7700::luxPerCount_(VEML7700_Gain g, VEML7700_IT it) {
  const float base = 0.0042f;
  const float IT = static_cast<float>(itMs_(it));
  const float G  = gainMul_(g);
  return base * (800.0f / IT) * (2.0f / G);
}
