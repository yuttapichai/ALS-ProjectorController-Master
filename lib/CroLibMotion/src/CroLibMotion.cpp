// #include "CroLibMotion.h"
// #include <string.h>

// namespace cro {

// uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
//   uint16_t crc = 0xFFFF;
//   for (size_t i = 0; i < len; ++i) {
//     crc ^= (uint16_t)data[i] << 8;
//     for (int b = 0; b < 8; ++b)
//       crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
//   }
//   return crc;
// }

// size_t computeFrameSize(uint8_t nm) {
//   return sizeof(FrameHeader) + (size_t)nm * sizeof(NodePayload50) + 2 + 2;
// }

// bool packFrame(const FrameHeader& hdr,
//                const NodePayload50* nodes, uint8_t nm,
//                uint8_t* out, size_t maxlen, size_t* out_len) {
//   if (!out || !out_len) return false;
//   if (!nodes && nm) return false;

//   const size_t total = computeFrameSize(nm);
//   if (maxlen < total) return false;

//   uint8_t* p = out;
//   memcpy(p, &hdr, sizeof(FrameHeader)); p += sizeof(FrameHeader);
//   if (nm) { memcpy(p, nodes, nm * sizeof(NodePayload50)); p += nm * sizeof(NodePayload50); }

//   const size_t span = sizeof(FrameHeader) + nm * sizeof(NodePayload50);
//   uint16_t crc = crc16_ccitt(out, span);
//   *(uint16_t*)p = crc; p += 2;
//   *(uint16_t*)p = CROMO_ETX; p += 2;

//   *out_len = (size_t)(p - out);
//   return true;
// }

// bool makeFrameMasked(uint16_t seq, uint16_t mask,
//                      const NodePayload50* nodes, uint8_t nm,
//                      uint8_t* out, size_t maxlen, size_t* out_len) {
//   FrameHeader h{};
//   h.stx = CROMO_STX;
//   h.ver = CROMO_VER;
//   h.seq = seq;
//   h.flags = 0;
//   h.mask = mask;
//   h.nm = nm;
//   h.t_ref_ms = 0;
//   return packFrame(h, nodes, nm, out, maxlen, out_len);
// }

// bool makeFrameTimed(uint16_t seq, uint32_t t_ref_ms,
//                     const NodePayload50* nodes, uint8_t nm,
//                     uint8_t* out, size_t maxlen, size_t* out_len) {
//   FrameHeader h{};
//   h.stx = CROMO_STX;
//   h.ver = CROMO_VER;
//   h.seq = seq;
//   h.flags = 0;
//   h.mask = 0;
//   h.nm = nm;
//   h.t_ref_ms = t_ref_ms;
//   return packFrame(h, nodes, nm, out, maxlen, out_len);
// }

// // =====================
// // Motion helpers
// // =====================
// MotorRecord16 motor::stepper_pos_abs(int32_t target_steps, uint8_t microstep,
//                                      int32_t vmax_pps, uint8_t flags) {
//   MotorRecord16 m{};
//   m.type=1; m.mode=1; m.opt=microstep; m.flags=flags;
//   m.p1=target_steps; m.p3=vmax_pps;
//   return m;
// }

// MotorRecord16 motor::stepper_pos_rel(int32_t delta_steps, uint8_t microstep,
//                                      int32_t vmax_pps, uint8_t flags) {
//   MotorRecord16 m{};
//   m.type=1; m.mode=2; m.opt=microstep; m.flags=flags;
//   m.p1=delta_steps; m.p3=vmax_pps;
//   return m;
// }

// MotorRecord16 motor::servo_angle_cdeg(int32_t angle_cdeg, int32_t speed_dps,
//                                       uint8_t flags) {
//   MotorRecord16 m{};
//   m.type=2; m.mode=1; m.opt=0; m.flags=flags;
//   m.p1=angle_cdeg; m.p3=speed_dps;
//   return m;
// }

// // =====================
// // Node builder
// // =====================
// NodePayload50 BaseHelper::makeNode(uint8_t node_id, uint8_t led,
//                                    const MotorRecord16& m1,
//                                    const MotorRecord16& m2) {
//   NodePayload50 np{};
//   np.node_id = node_id;
//   np.led = led;
//   np.m[0] = m1;
//   np.m[1] = m2;
//   return np;
// }

// } // namespace cro
// // =====================
// // End of CroLibMotion.cpp
// // =====================