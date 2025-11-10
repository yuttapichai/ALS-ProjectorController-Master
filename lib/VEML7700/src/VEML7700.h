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

#pragma once
#include <Arduino.h>
#include <Wire.h>

// 7-bit address
#ifndef VEML7700_ADDR
#define VEML7700_ADDR 0x10
#endif

// Register map
#define REG_ALS_CONF_0  0x00
#define REG_ALS_WH      0x01
#define REG_ALS_WL      0x02
#define REG_PSM         0x03
#define REG_ALS_DATA    0x04
#define REG_WHITE_DATA  0x05
#define REG_ALS_INT     0x06
#define REG_ID          0x07

// Gain & IT enums (datasheet)
enum VEML7700_Gain : uint8_t {
  GAIN_X1   = 0b00,
  GAIN_X2   = 0b01,
  GAIN_X1_8 = 0b10,
  GAIN_X1_4 = 0b11
};

enum VEML7700_IT : uint8_t {
  IT_25ms  = 0b1100,
  IT_50ms  = 0b1000,
  IT_100ms = 0b0000,
  IT_200ms = 0b0001,
  IT_400ms = 0b0010,
  IT_800ms = 0b0011
};

struct VEML7700_Config {
  VEML7700_Gain gain = GAIN_X2;
  VEML7700_IT   it   = IT_800ms;
  uint8_t       pers = 0b00;     // 1,2,4,8
  bool          int_en = false;  // ALS_INT_EN
  bool          shutdown = false;
};

class VEML7700 {
public:
  explicit VEML7700(TwoWire &wire = Wire, uint8_t i2c_addr = VEML7700_ADDR);

  // Init I2C (ESP32: begin(sda,scl,freq)). If you already began Wire elsewhere, set sda/scl to -1.
  bool begin(const VEML7700_Config &cfg,
             int sda = -1, int scl = -1, uint32_t freq = 400000);

  // Configuration helpers
  bool applyConfig(const VEML7700_Config &cfg);
  void setCalibrationFactor(float cf) { cal_factor_ = cf; }
  float calibrationFactor() const { return cal_factor_; }

  // Raw reads
  bool readALS(uint16_t &rawALS);
  bool readWHITE(uint16_t &rawWHITE);
  bool readID(uint16_t &id);

  // Lux (applies Gain/IT scaling + user calibration factor)
  bool readLux(float &lux) const;

  // Quick setters (re-writes config register)
  bool setGain(VEML7700_Gain g);
  bool setIntegrationTime(VEML7700_IT it);

  // Current config (cached)
  VEML7700_Config config() const { return cfg_; }

private:
  // I2C
  bool write16_(uint8_t reg, uint16_t value) const;
  bool read16_(uint8_t reg, uint16_t &out) const;

  // Helpers
  static uint16_t packConf0_(const VEML7700_Config &c);
  static uint16_t itMs_(VEML7700_IT it);
  static float gainMul_(VEML7700_Gain g);
  static float luxPerCount_(VEML7700_Gain g, VEML7700_IT it);

private:
  TwoWire     *wire_;
  uint8_t      addr_;
  VEML7700_Config cfg_;
  float        cal_factor_ = 1.0f;  // default 1.0; user can set e.g. 0.33
};
