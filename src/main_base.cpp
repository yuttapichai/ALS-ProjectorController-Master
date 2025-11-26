// ===================== main_base.cpp =====================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "CroLibMotion.h"
#include "ESPNowBase.h"
#include "VEML7700.h"
#include "config.h"
//* -------------------- VEML7700 light sensor --------------------
VEML7700 sensor;
VEML7700_Config cfg;

// -------------------- ESP-NOW base --------------------
ESPNowBaseController Base;

static const uint8_t NODE_MAC[5][6] = {
    {0x80, 0xB5, 0x4E, 0xC2, 0xF7, 0xF4}, // node1
    {0xD0, 0xCF, 0x13, 0x15, 0x12, 0x74}, // node2
    {0xD0, 0xCF, 0x13, 0x15, 0x39, 0x9C}, // node3
    {0xD0, 0xCF, 0x13, 0x16, 0x2C, 0x74}, // node4
    {0xD0, 0xCF, 0x13, 0x16, 0xDB, 0x1C}, // node5
};

static void onSendCb(const uint8_t *mac, esp_now_send_status_t st)
{
  Serial.printf("[Base] send_cb %02X:%02X:%02X:%02X:%02X:%02X -> %s\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                st == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// -------------------- Helpers --------------------
inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi)
{
  return (v < lo ? lo : (v > hi ? hi : v));
}
inline uint8_t clampU8(uint8_t v, uint8_t lo, uint8_t hi)
{
  return (v < lo ? lo : (v > hi ? hi : v));
}

// pick the k-th O/X/F character from string
static char pickOXF(const char *s, uint8_t k)
{
  uint8_t seen = 0;
  for (; *s; ++s)
  {
    char c = *s;
    if (c == 'O' || c == 'X' || c == 'Y' || c == 'F')
    {
      if (++seen == k)
        return c;
    }
  }
  return 'F';
}

// save last direction per motor per node
static uint8_t g_dir_last[6][2] = {{0}}; // 0=CW, 1=CCW

// -------------------------- define sequence --------------------------
uint8_t sequence_selection = 1; // 1 or 2

// set both motors of a node
static inline void setNode(cro::MotionFrame &f, int idx,
                           uint8_t s1, uint16_t d1, uint8_t t1, uint8_t dir1,
                           uint8_t s2, uint16_t d2, uint8_t t2, uint8_t dir2)
{
  cro::setMotor(f, idx, 0, s1, d1, t1, dir1);
  cro::setMotor(f, idx, 1, s2, d2, t2, dir2);
}

// map O/X/F → action (motor)
static inline void applyMotorOXF(cro::MotionFrame &f, int nodeIdx, int mIdx,
                                 char ch, const MotorPreset &P)
{
  uint8_t dir = g_dir_last[nodeIdx][mIdx];
  switch (ch)
  {
  case 'O':
    dir = 0; // CW
    cro::setMotor(f, nodeIdx, mIdx, P.speed, P.degree, P.times, dir);
    break;
  case 'X':
    dir = 2; // Toggle
    cro::setMotor(f, nodeIdx, mIdx, P.speed, P.degree, P.times, dir);
    break;
  case 'F':
  default:
    cro::setMotor(f, nodeIdx, mIdx, 0, 0, 0, dir); // Store last direction
    break;
  }
  g_dir_last[nodeIdx][mIdx] = dir;
}

// map O/X/F → LedMode per Node
static inline cro::LedMode mapLedOXF(char c)
{
  if (c == 'O')
    return cro::LED_ON;
  if (c == 'X')
    return cro::LED_DIM_SLOW;
  if (c == 'Y')
    return cro::LED_DIM_FAST;
  return cro::LED_OFF; // 'F'
}

// build pattern #1 frame
static cro::MotionFrame buildPattern1(uint8_t seq /*1..19*/)
{
  seq = clampU8(seq, 1, PATTERN_NUM_SEQUENCES);

  cro::MotionFrame f = cro::makeFrame(/*bitmask*/ 0x3F, /*motorType*/ 1); // Stepper

  // LED per-node
  for (int n = 0; n < 6; ++n)
  {
    char cLED = pickOXF(P1_LED[n], seq);
    cro::setNodeLed(f, n, mapLedOXF(cLED));
  }

  // M1/M2 per-node
  for (int n = 0; n < 6; ++n)
  {
    char c1 = pickOXF(P1_M1[n], seq);
    char c2 = pickOXF(P1_M2[n], seq);
    applyMotorOXF(f, n, 0, c1, M1_PRESET);
    applyMotorOXF(f, n, 1, c2, M2_PRESET);
  }

  return f;
}

// build pattern #2 frame
static cro::MotionFrame buildPattern2(uint8_t seq /*1..19*/)
{
  seq = clampU8(seq, 1, PATTERN_NUM_SEQUENCES);

  cro::MotionFrame f = cro::makeFrame(/*bitmask*/ 0x3F, /*motorType*/ 1); // Stepper

  // LED per-node
  for (int n = 0; n < 6; ++n)
  {
    char cLED = pickOXF(P2_LED[n], seq);
    cro::setNodeLed(f, n, mapLedOXF(cLED));
  }

  // M1/M2 per-node
  for (int n = 0; n < 6; ++n)
  {
    char c1 = pickOXF(P2_M1[n], seq);
    char c2 = pickOXF(P2_M2[n], seq);
    applyMotorOXF(f, n, 0, c1, M1_PRESET);
    applyMotorOXF(f, n, 1, c2, M2_PRESET);
  }

  return f;
}

// build pattern #3 frame
static cro::MotionFrame buildPattern3(uint8_t seq /*1..19*/)
{
  seq = clampU8(seq, 1, PATTERN_NUM_SEQUENCES);

  cro::MotionFrame f = cro::makeFrame(/*bitmask*/ 0x3F, /*motorType*/ 1); // Stepper

  // LED per-node
  for (int n = 0; n < 6; ++n)
  {
    char cLED = pickOXF(P3_LED[n], seq);
    cro::setNodeLed(f, n, mapLedOXF(cLED));
  }

  // M1/M2 per-node
  for (int n = 0; n < 6; ++n)
  {
    char c1 = pickOXF(P3_M1[n], seq);
    char c2 = pickOXF(P3_M2[n], seq);
    applyMotorOXF(f, n, 0, c1, M1_PRESET);
    applyMotorOXF(f, n, 1, c2, M2_PRESET);
  }

  return f;
}

// ===================== Command Task =====================
static void taskCommand(void *pv)
{
  uint8_t seq = 0; // 1..19
  for (;;)
  {
    seq = (seq % PATTERN_NUM_SEQUENCES) + 1;

    if (sequence_selection == 2)
    {
      auto frame = buildPattern2(seq);
      cro::printFrame(frame);
      bool ok = Base.sendFrame(frame);
      Serial.printf("[Base] send P#2 seq=%u -> %s\n", seq, ok ? "OK" : "FAIL");
    }
    else if (sequence_selection == 1)
    {
      auto frame = buildPattern1(seq);
      cro::printFrame(frame);
      bool ok = Base.sendFrame(frame);
      Serial.printf("[Base] send P#1 seq=%u -> %s\n", seq, ok ? "OK" : "FAIL");
    }
    else if (sequence_selection == 3)
    {
      auto frame = buildPattern3(seq);
      cro::printFrame(frame);
      bool ok = Base.sendFrame(frame);
      Serial.printf("[Base] send P#3 seq=%u -> %s\n", seq, ok ? "OK" : "FAIL");
    }
    vTaskDelay(pdMS_TO_TICKS(PATTERN_TIMEFRAME_MS));
  }
}

// ===================== Sensor Task =====================
static void taskSensor(void *pv)
{
  for (;;)
  {
    float lux = 0.0f;
    if (sensor.readLux(lux))
    {
      Serial.printf("[Base] Ambient Light: %.2f lux\n", lux);
      if (lux <= LIGHT_SENSOR_THRESHOLD_LUX_MIN)
      {
        sequence_selection = 1;
      }
      else 
      if (lux <= LIGHT_SENSOR_THRESHOLD_LUX_MID)
      {
        sequence_selection = 2;
      }
      else if (lux > LIGHT_SENSOR_THRESHOLD_LUX_MAX)
      {
        sequence_selection = 3;
      }
    }
    else
    {
      Serial.println("[Base] Failed to read lux from VEML7700");
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ===================== setup / loop =====================
void setup()
{
  Serial.begin(115200);
  delay(300);

  // Lowest sensitivity and fastest IT
  cfg.gain = GAIN_X1;
  cfg.it = IT_100ms;
  cfg.pers = 0b00;
  cfg.int_en = false;
  cfg.shutdown = false;

  if (!sensor.begin(cfg, /*sda=*/8, /*scl=*/9, /*freq=*/400000))
  {
    Serial.println("VEML7700 init failed");
    // while (1)
    delay(1000);
  }

  sensor.setCalibrationFactor(0.38f);

  Serial.println("VEML7700 init OK");

  Serial.println("\n=== Base (Active) ===");

  Base.begin(1);
  esp_now_register_send_cb(onSendCb);

  xTaskCreatePinnedToCore(taskCommand, "taskCommand", 6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSensor, "taskSensor", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(100));
}
