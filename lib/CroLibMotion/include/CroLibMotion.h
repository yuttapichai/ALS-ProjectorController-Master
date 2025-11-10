#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace cro {

// ===== Constants =====
constexpr uint8_t  CROMO_STX          = 0x02;
constexpr uint8_t  CROMO_ETX          = 0x03;
constexpr uint8_t  CROMO_NODE_MAX     = 6;   // Node 1..6
constexpr uint8_t  CROMO_MOTOR_PER_NODE = 2; // M1..M2

// LED modes (per node)
enum LedMode : uint8_t {
  LED_OFF      = 0,
  LED_ON       = 1,
  LED_DIM_SLOW = 2,
  LED_DIM_FAST = 3,
};

// ===== Payload structs (packed เพื่อกัน padding) =====
struct MotorData {
  uint8_t  speed;     // 0..100 (%)
  uint16_t degree;    // 0..360 (หรือมากกว่าได้ถึง 65535)
  uint8_t  times;     // 0..99
  uint8_t  dir;       // 0=CW, 1=CCW
} __attribute__((packed));                     // 6 bytes

struct NodeData {
  uint8_t   led;      // LedMode (per node)
  MotorData m1;       // 6 bytes
  MotorData m2;       // 6 bytes
} __attribute__((packed));                     // 13 bytes

// ===== Main Frame (with per-node LED) =====
// Size = 1(STX)+1(bitmask)+1(motorType)+ (6*13) +1(ETX) = 82 bytes
struct MotionFrame {
  uint8_t  stx;                       // 0x02
  uint8_t  bitmask;                   // bit0→Node1 ... bit5→Node6
  uint8_t  motorType;                 // 0=Servo, 1=Stepper
  NodeData node[CROMO_NODE_MAX];      // per-node LED + 2 motors
  uint8_t  etx;                       // 0x03
} __attribute__((packed));

// ===== Builders / Helpers =====

// เวอร์ชันหลัก: ไม่กำหนด LED (ผู้ใช้ไปตั้งทีหลัง, ต่อ node)
inline MotionFrame makeFrame(uint8_t bitmask, uint8_t motorType) {
  MotionFrame f{};
  f.stx       = CROMO_STX;
  f.bitmask   = bitmask;
  f.motorType = motorType;
  f.etx       = CROMO_ETX;
  return f;
}

// เวอร์ชันอำนวยความสะดวก: ตั้ง LED เหมือนกันทุก node
inline MotionFrame makeFrame(uint8_t bitmask, LedMode allNodeLed, uint8_t motorType) {
  MotionFrame f = makeFrame(bitmask, motorType);
  for (int i = 0; i < CROMO_NODE_MAX; ++i) f.node[i].led = (uint8_t)allNodeLed;
  return f;
}

// ตั้งค่า LED ต่อ node
inline void setNodeLed(MotionFrame& f, uint8_t nodeIndex, LedMode led) {
  if (nodeIndex >= CROMO_NODE_MAX) return;
  f.node[nodeIndex].led = (uint8_t)led;
}

// ตั้งค่า LED ทุก node
inline void setAllLeds(MotionFrame& f, LedMode led) {
  for (int i = 0; i < CROMO_NODE_MAX; ++i) f.node[i].led = (uint8_t)led;
}

// ตั้งค่ามอเตอร์ของ nodeIndex (0..5), motorIndex (0=M1, 1=M2)
inline void setMotor(MotionFrame &f, uint8_t nodeIndex, uint8_t motorIndex,
                     uint8_t speed, uint16_t degree, uint8_t times, uint8_t dir) {
  if (nodeIndex >= CROMO_NODE_MAX || motorIndex > 1) return;
  MotorData &m = (motorIndex == 0) ? f.node[nodeIndex].m1 : f.node[nodeIndex].m2;
  m.speed  = speed;
  m.degree = degree;
  m.times  = times;
  m.dir    = dir;
  // m.dir    = dir ? 1 : 0;
}

// Serialize / Parse
inline size_t serializeFrame(const MotionFrame &f, uint8_t *out, size_t maxlen) {
  if (!out || maxlen < sizeof(MotionFrame)) return 0;
  memcpy(out, &f, sizeof(MotionFrame));
  return sizeof(MotionFrame);
}
inline bool parseFrame(const uint8_t *data, size_t len, MotionFrame *out) {
  if (!data || !out || len < sizeof(MotionFrame)) return false;
  if (data[0] != CROMO_STX || data[sizeof(MotionFrame)-1] != CROMO_ETX) return false;
  memcpy(out, data, sizeof(MotionFrame));
  return true;
}

// Debug print
inline const char* ledName(uint8_t m) {
  switch (m) { case LED_OFF: return "OFF"; case LED_ON: return "ON";
               case LED_DIM_SLOW: return "DIM_SLOW"; case LED_DIM_FAST: return "DIM_FAST"; }
  return "?";
}
inline void printFrame(const MotionFrame &f) {
  printf("\n[CroMotion] mask=0x%02X  motorType=%s  size=%uB\n",
         f.bitmask, f.motorType ? "Stepper":"Servo", (unsigned)sizeof(MotionFrame));
  for (int n = 0; n < CROMO_NODE_MAX; ++n) {
    printf(" Node%u LED=%s | "
           "M1: spd=%3u deg=%3u t=%2u dir=%s | "
           "M2: spd=%3u deg=%3u t=%2u dir=%s\n",
           n+1, ledName(f.node[n].led),
           f.node[n].m1.speed, f.node[n].m1.degree, f.node[n].m1.times, (f.node[n].m1.dir == 2) ? "WAVE":(f.node[n].m1.dir == 1) ?"CCW":"CW",
           f.node[n].m2.speed, f.node[n].m2.degree, f.node[n].m2.times, (f.node[n].m2.dir == 2) ? "WAVE":(f.node[n].m2.dir == 1) ?"CCW":"CW");
  }
  printf("-----------------------------------------------------\n");
}

} // namespace cro
