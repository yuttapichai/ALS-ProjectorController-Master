#include <Arduino.h>
#include <CroLibMotion.h>

using namespace cro;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("CroLibMotion example");

  // Build a stepper and a servo record
  auto step = motor::stepper_pos_abs(160000, 16, 20000, 10000);
  auto serv = motor::servo_angle_cdeg(9000, 120, 70);

  // Show CRC over these two records (demo only)
  uint8_t buf[sizeof(MotorRecord16) * 2];
  memcpy(buf, &step, sizeof(step));
  memcpy(buf + sizeof(step), &serv, sizeof(serv));
  uint16_t crc = crc16_ccitt(buf, sizeof(buf));
  Serial.printf("CRC16 over two records = 0x%04X\n", crc);
}

void loop() {
  delay(1000);
}