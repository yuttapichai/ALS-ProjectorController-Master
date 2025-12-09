#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "CroLibMotion.h"
using namespace cro;

// ---------- CONFIG ----------
#define NODE_ID 4
#define ESPNOW_CH 1

// LED (PWM)
#define PWM_PIN 4
#define PWM_CH 0
#define PWM_RES 12
#define PWM_FREQ 5000

#define M1_STEP_PIN 5
#define M1_DIR_PIN 6
#define M1_EN_PIN 7

#define M2_STEP_PIN 8
#define M2_DIR_PIN 3
#define M2_EN_PIN 46

#define M1_LIMIT_SWITCH_PIN 1
#define M2_LIMIT_SWITCH_PIN 2

#define PULLEY_RATIO (18.34 / 10.7) // mm.

static const uint16_t FULL_STEPS_PER_REV = 200;
static const uint16_t MICROSTEPS = 8;

static const uint32_t PPS_MIN = (uint32_t)(10 * MICROSTEPS * PULLEY_RATIO);
static const uint32_t PPS_MAX = (uint32_t)(50 * MICROSTEPS * PULLEY_RATIO);

#define M2_WAVE_PULSE_CW 200
#define M2_WAVE_PULSE_CCW 200
#define M2_WAVE_HALF_US 2000

// ====== Analog limit thresholds (millivolts) ======
static const uint16_t M2_ACTIVE_MV_MAX = 300;
static const uint16_t M2_INACTIVE_MV_MIN = 1800;

// ---------- Queues / Tasks ----------
struct FrameWrap
{
  cro::MotionFrame f;
};
static QueueHandle_t rxQ = nullptr;

enum CmdType : uint8_t
{
  CMD_ADD,
  CMD_STOP
};
struct MotorCmd
{
  CmdType type;
  cro::MotorData md;
};
static QueueHandle_t motorQ1 = nullptr;
static QueueHandle_t motorQ2 = nullptr;
static TaskHandle_t hMotor1 = nullptr;
static TaskHandle_t hMotor2 = nullptr;

// ---------- LED state ----------
static volatile uint8_t LED_MODE = 0; // 0=OFF,1=ON,2=DIM_SLOW,3=DIM_FAST
static volatile bool LED_MODE_UPDATED = false;

// ---------- M1 runtime position & home state ----------
static volatile int32_t g_M1_pos_steps = 0;      // logical position of M1 in microsteps
static volatile bool g_M1_home_latched = false;  // set when M1 hits home while running
static bool g_M1_home_prev = false;              // last home state for edge detect

// (Optional) M2 position if needed later
static volatile int32_t g_M2_pos_steps = 0;

// ---------- Utils ----------
static const char *ledModeName(uint8_t v)
{
  switch (v)
  {
  case LED_OFF:
    return "OFF";
  case LED_ON:
    return "ON";
  case LED_DIM_SLOW:
    return "DIM_SLOW";
  case LED_DIM_FAST:
    return "DIM_FAST";
  }
  return "?";
}
static inline const char *dirName(uint8_t d) { return (d == 2) ? "WAVE" : (d == 1) ? "CCW"
                                                                                   : "CW"; }

static inline uint32_t mapSpeedToPPS(uint8_t speed)
{
  if (!speed)
    return 0;
  if (speed > 100)
    speed = 100;
  return PPS_MIN + (uint32_t)speed * (PPS_MAX - PPS_MIN) / 100u;
}
static inline uint32_t degreeToPulses(uint16_t degree, uint8_t times)
{
  if (!degree || !times)
    return 0;
  const uint32_t stepsPerRev = (uint32_t)FULL_STEPS_PER_REV * MICROSTEPS;
  const uint32_t perCycle = (uint32_t)round(degree * stepsPerRev / 360.0f);
  return perCycle * (uint32_t)times;
}

// ---------- ESP-NOW RX ----------
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
  // version check
  if (len != (int)sizeof(cro::MotionFrame))
  {
    Serial.printf("[Node%u] DROP size: got=%d expect=%u\n", NODE_ID, len, (unsigned)sizeof(cro::MotionFrame));
    return;
  }
  if (data[0] != CROMO_STX || data[len - 1] != CROMO_ETX)
  {
    Serial.printf("[Node%u] DROP STX/ETX: stx=0x%02X etx=0x%02X\n", NODE_ID, data[0], data[len - 1]);
    return;
  }
  FrameWrap it{};
  memcpy(&it.f, data, sizeof(it.f));
  if (rxQ)
    xQueueSend(rxQ, &it, 0);
}

/**
 * ====== M2 analog limit read with hysteresis + debounce ======
 */
static bool s_m2Active = false;

static inline int readM2_mV_once()
{
  return analogReadMilliVolts(M2_LIMIT_SWITCH_PIN);
}

// debounce
static bool confirmState(bool wantActive, uint8_t samples = 5)
{
  uint8_t ok = 0;
  for (uint8_t i = 0; i < samples; ++i)
  {
    int mv = readM2_mV_once();
    if (wantActive)
    {
      if (mv < M2_ACTIVE_MV_MAX)
        ++ok;
    }
    else
    {
      if (mv > M2_INACTIVE_MV_MIN)
        ++ok;
    }
    delayMicroseconds(200);
  }
  return ok >= (samples - 1);
}

static bool isM2Active()
{
  if (s_m2Active)
  {

    if (confirmState(false))
      s_m2Active = false;
  }
  else
  {

    if (confirmState(true))
      s_m2Active = true;
  }
  return s_m2Active;
}
/**
 * ======================================================================
 */

// ---------- TASK: process incoming frames ----------
static void taskRxProcess(void *)
{
  const uint8_t idx = NODE_ID - 1;
  FrameWrap it{};
  for (;;)
  {
    if (xQueueReceive(rxQ, &it, portMAX_DELAY) != pdTRUE)
      continue;

    if ((it.f.bitmask & (1u << idx)) == 0)
    {
      if (LED_MODE != LED_OFF)
      {
        LED_MODE = LED_OFF;
        LED_MODE_UPDATED = true;
      }
      continue;
    }

    const NodeData &nd = it.f.node[idx];

    if (nd.led != LED_MODE)
    {
      LED_MODE = nd.led;
      LED_MODE_UPDATED = true;
    }

    const MotorData &m1 = nd.m1;
    const MotorData &m2 = nd.m2;

    Serial.println("\n================= CroMotion RX =================");
    Serial.printf("[Node%u] mask=0x%02X  motorType=%s  size=%uB\n",
                  NODE_ID, it.f.bitmask, it.f.motorType ? "Stepper" : "Servo", (unsigned)sizeof(cro::MotionFrame));
    Serial.printf(" LED=%u (%s)\n", (unsigned)nd.led, ledModeName(nd.led));
    Serial.printf(" M1: spd=%u deg=%u times=%u dir=%s\n",
                  (unsigned)m1.speed, (unsigned)m1.degree, (unsigned)m1.times, dirName(m1.dir));
    Serial.printf(" M2: spd=%u deg=%u times=%u dir=%s\n",
                  (unsigned)m2.speed, (unsigned)m2.degree, (unsigned)m2.times, dirName(m2.dir));
    Serial.println("================================================");

    auto toCmd = [](const cro::MotorData &md) -> MotorCmd
    {
      MotorCmd c{};
      c.md = md;
      c.type = (md.speed == 0 || md.times == 0) ? CMD_STOP : CMD_ADD;
      return c;
    };
    MotorCmd c1 = toCmd(m1);
    MotorCmd c2 = toCmd(m2);

    xQueueSend(motorQ1, &c1, pdMS_TO_TICKS(10));
    xQueueSend(motorQ2, &c2, pdMS_TO_TICKS(10));

    // wake up motor tasks
    if (hMotor1)
      xTaskNotifyGive(hMotor1);
    if (hMotor2)
      xTaskNotifyGive(hMotor2);
  }
}

// ---------- TASK: LED dimming ----------
static void taskHeartbeat(void *)
{
  const uint16_t PWM_MAX = (1u << PWM_RES) - 1u;
  uint16_t value = 0;
  bool fadeUp = true;
  uint8_t lastMode = 0xFF;
  uint8_t fast_dim_step = 0;
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
  ledcWrite(PWM_CH, 0);

  const uint32_t T_SLOW_MS = 1500, T_FAST_MS = 500;
  const uint16_t DELAY_SLOW_MS = 10, DELAY_FAST_MS = 5;
  auto stepCalc = [&](uint32_t period, uint16_t dly)
  {
    uint32_t N = (period && dly) ? (period / dly) : 1;
    if (N < 2)
      N = 2;
    uint32_t s = (2u * PWM_MAX) / N;
    if (!s)
      s = 1;
    if (s > PWM_MAX)
      s = PWM_MAX;
    return (uint16_t)s;
  };

  for (;;)
  {
    uint8_t mode = LED_MODE;
    if (LED_MODE_UPDATED || mode != lastMode)
    {
      lastMode = mode;
      LED_MODE_UPDATED = false;
      fadeUp = true;
      value = (mode == LED_ON) ? PWM_MAX : 0;
      fast_dim_step = 0;
      ledcWrite(PWM_CH, value);
      Serial.printf("[Node%u] LED -> %s\n", NODE_ID, ledModeName(mode));
    }
    switch (mode)
    {
    case LED_OFF:
      ledcWrite(PWM_CH, 0);
      value = 0;
      fadeUp = true;
      vTaskDelay(pdMS_TO_TICKS(20));
      break;
    case LED_ON:
      ledcWrite(PWM_CH, PWM_MAX);
      value = PWM_MAX;
      vTaskDelay(pdMS_TO_TICKS(20));
      break;
    case LED_DIM_SLOW:
    {
      uint16_t s = stepCalc(T_SLOW_MS, DELAY_SLOW_MS);
      ledcWrite(PWM_CH, value);
      if (fadeUp)
      {
        if (value + s >= PWM_MAX)
        {
          value = PWM_MAX;
          fadeUp = false;
        }
        else
          value += s;
      }
      else
      {
        if (value <= s)
        {
          value = 0;
          fadeUp = true;
        }
        else
          value -= s;
      }
      vTaskDelay(pdMS_TO_TICKS(DELAY_SLOW_MS));
      break;
    }
    case LED_DIM_FAST:
    {
      if (fast_dim_step < 30)
      {
        value = PWM_MAX;
        ++fast_dim_step;
      }
      else if (fast_dim_step < 60)
      {
        value = 0;
        ++fast_dim_step;
      }
      else if (fast_dim_step < 90)
      {
        value = PWM_MAX;
        ++fast_dim_step;
      }
      else if (fast_dim_step < 120)
      {
        value = 0;
        ++fast_dim_step;
      }
      else if (fast_dim_step < 140)
      {
        value = PWM_MAX;
        ++fast_dim_step;
      }
      else if (fast_dim_step <= 180)
      {
        value = 0;
        ++fast_dim_step;
      }
      else if (fast_dim_step > 180)
      {
        value = PWM_MAX;
      }
      ledcWrite(PWM_CH, value);
      vTaskDelay(pdMS_TO_TICKS(DELAY_FAST_MS));
      break;
    }
    default:
      ledcWrite(PWM_CH, 0);
      vTaskDelay(pdMS_TO_TICKS(20));
      break;
    }
  }
}

// ---------- Accumulator driver ----------
struct MotorRun
{
  volatile uint32_t pending[2] = {0, 0}; // 0=CW, 1=CCW
  volatile uint32_t pps = 400;
  volatile uint8_t cur_dir = 0;
  volatile bool enabled = false;
};

static void applyCmd(MotorRun &r, const MotorCmd &cmd)
{
  if (cmd.type == CMD_STOP)
  {
    r.pending[0] = r.pending[1] = 0;
    r.enabled = false;
    return;
  }
  uint32_t add = degreeToPulses(cmd.md.degree, cmd.md.times);
  if (!add)
    return;
  r.pps = mapSpeedToPPS(cmd.md.speed);
  r.pending[(cmd.md.dir == 1) ? 1 : 0] += add;
  r.enabled = true;
  // cmd.md.dir [0,..,2]: 0=CW,1=CCW,2=Toggle
  r.cur_dir = cmd.md.dir;
}

static void applyCmd_M2(MotorRun &r, const MotorCmd &cmd)
{
  if (cmd.type == CMD_STOP)
  {
    r.pending[0] = r.pending[1] = 0;
    r.enabled = false;
    return;
  }

  if (cmd.md.dir == 2)
  {
    r.cur_dir = 2;
    r.enabled = true;
    return;
  }

  uint32_t add = degreeToPulses(cmd.md.degree, cmd.md.times);
  if (!add)
    return;

  r.pps = mapSpeedToPPS(cmd.md.speed);
  uint8_t idx = (cmd.md.dir == 1) ? 1 : 0; // 0=CW, 1=CCW
  r.pending[idx] += add;
  r.enabled = true;
  r.cur_dir = idx;
}

// ---------- Pulse generator with M1 home check ----------
static void pulseChunk(uint8_t m_number, uint8_t stepPin, uint8_t dirPin, int8_t enPin,
                       uint8_t dir, uint32_t n, uint32_t halfUs)
{
  // M1: normal CW/CCW with home detect and position tracking
  if (m_number == 1)
  {
    // For M1: dir==0 => CW, dir==1 => CCW (mapping kept same as previous code)
    digitalWrite(dirPin, dir ? LOW : HIGH); // HIGH CW, LOW CCW

    if (enPin >= 0)
      digitalWrite((uint8_t)enPin, LOW); // enable (active-low)

    const uint32_t YIELD_EVERY = 256;
    bool homePrev = g_M1_home_prev;

    for (uint32_t i = 0; i < n; ++i)
    {
      digitalWrite(stepPin, HIGH);
      ets_delay_us(halfUs);
      digitalWrite(stepPin, LOW);
      ets_delay_us(halfUs);

      // Update logical position: CW=+1, CCW=-1
      if (dir == 0)
        g_M1_pos_steps++;
      else
        g_M1_pos_steps--;

      // Home switch is active-low
      bool homeNow = (digitalRead(M1_LIMIT_SWITCH_PIN) == LOW);
      if (!homePrev && homeNow)
      {
        // Just entered home -> latch and reset position
        g_M1_home_latched = true;
        g_M1_pos_steps = 0;
        homePrev = homeNow;
        break;
      }
      homePrev = homeNow;

      if ((i % YIELD_EVERY) == 0)
        vTaskDelay(0);
    }

    g_M1_home_prev = homePrev;

    if (enPin >= 0)
      digitalWrite((uint8_t)enPin, HIGH); // disable (active-low)
    return;
  }
  // M2 with special WAVE mode
  else if (m_number == 2)
  {
    if (m_number == 2 && dir == 2)
    {
      if (enPin >= 0)
        digitalWrite((uint8_t)enPin, LOW);

      // CW 200
      digitalWrite(dirPin, HIGH);
      for (uint32_t i = 0; i < M2_WAVE_PULSE_CW; ++i)
      {
        digitalWrite(stepPin, HIGH);
        ets_delay_us(M2_WAVE_HALF_US);
        digitalWrite(stepPin, LOW);
        ets_delay_us(M2_WAVE_HALF_US);
        if ((i & 0xFF) == 0)
          vTaskDelay(0);
      }
      // CCW 200
      digitalWrite(dirPin, LOW);
      for (uint32_t i = 0; i < M2_WAVE_PULSE_CCW; ++i)
      {
        digitalWrite(stepPin, HIGH);
        ets_delay_us(M2_WAVE_HALF_US);
        digitalWrite(stepPin, LOW);
        ets_delay_us(M2_WAVE_HALF_US);
        if ((i & 0xFF) == 0)
          vTaskDelay(0);
      }

      if (enPin >= 0)
        digitalWrite((uint8_t)enPin, HIGH);
      return;
    }
    else
    {
      // For M2: dir==0 => CW, dir==1 => CCW
      digitalWrite(dirPin, dir ? HIGH : LOW); // LOW CW, HIGH CCW
    }
  }

  // Generic chunk (used by M2 CW/CCW)
  if (enPin >= 0)
    digitalWrite((uint8_t)enPin, LOW); // enable (active-low)
  const uint32_t YIELD_EVERY = 256;
  for (uint32_t i = 0; i < n; ++i)
  {
    digitalWrite(stepPin, HIGH);
    ets_delay_us(halfUs);
    digitalWrite(stepPin, LOW);
    ets_delay_us(halfUs);

    if (m_number == 2)
    {
      // Optional: track M2 position if needed (similar to M1)
      if (dir == 0)
        g_M2_pos_steps++;
      else
        g_M2_pos_steps--;
    }

    if ((i % YIELD_EVERY) == 0)
      vTaskDelay(0);
  }
  if (enPin >= 0)
    digitalWrite((uint8_t)enPin, HIGH);
}

// M1
static void taskMotor1(void *)
{
  MotorRun R;
  MotorCmd cmd;

  for (;;)
  {
    while (xQueueReceive(motorQ1, &cmd, 0) == pdTRUE)
      applyCmd(R, cmd);

    if (!R.enabled || (R.pending[0] == 0 && R.pending[1] == 0))
    {
      R.enabled = false;
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }

    uint8_t d = R.cur_dir;
    uint32_t pps = R.pps ? R.pps : 200;
    uint32_t halfUs = (uint32_t)(500000UL / pps);
    uint32_t chunk = R.pending[d];
    if (chunk > 512)
      chunk = 512;

    // Remember previous home latch state (should normally be false)
    bool wasHomeLatched = g_M1_home_latched;

    pulseChunk(1, M1_STEP_PIN, M1_DIR_PIN,
#ifdef M1_EN_PIN
               M1_EN_PIN,
#else
               -1,
#endif
               d, chunk, halfUs);

    // If M1 hit home during this chunk, stop immediately and clear queue
    if (g_M1_home_latched && !wasHomeLatched)
    {
      Serial.printf("[Node%u][M1] HOME hit while running -> stop & reset pos=0\n", NODE_ID);

      R.pending[0] = R.pending[1] = 0;
      R.enabled = false;

      // Flush any pending commands in the queue
      while (xQueueReceive(motorQ1, &cmd, 0) == pdTRUE)
      {
        // discard commands on HOME hit
      }

      // Clear latch after handling
      g_M1_home_latched = false;

      // Go wait for next command
      continue;
    }

    // Normal case: no home hit, reduce pending by executed chunk
    R.pending[d] -= chunk;

    while (xQueueReceive(motorQ1, &cmd, 0) == pdTRUE)
      applyCmd(R, cmd);

    if (R.pending[d] == 0 && R.pending[d ^ 1] > 0)
      R.cur_dir = d ^ 1;
  }
}

// M2
static void taskMotor2(void *)
{
  MotorRun R;
  MotorCmd cmd;

  for (;;)
  {

    while (xQueueReceive(motorQ2, &cmd, 0) == pdTRUE)
      applyCmd_M2(R, cmd);

    if (!R.enabled || (R.pending[0] == 0 && R.pending[1] == 0 && R.cur_dir != 2))
    {
      R.enabled = false;
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }

    // mode wave
    if (R.cur_dir == 2)
    {
      const uint32_t halfUs_wave = 2000; // = 250 pps

      pulseChunk(2, M2_STEP_PIN, M2_DIR_PIN,
#ifdef M2_EN_PIN
                 M2_EN_PIN,
#else
                 -1,
#endif
                 2 /*dir X*/, 0 /*n unused*/, halfUs_wave);

      R.cur_dir = 0;

      if (R.pending[0] == 0 && R.pending[1] == 0)
      {
        R.enabled = false;
      }

      while (xQueueReceive(motorQ2, &cmd, 0) == pdTRUE)
        applyCmd_M2(R, cmd);

      continue;
    }

    // ===== mode CW/CCW =====
    uint8_t d_idx = (R.cur_dir == 1) ? 1 : 0;
    uint32_t pps = R.pps ? R.pps : 200;
    uint32_t halfUs = (uint32_t)(500000UL / pps);

    uint32_t chunk = R.pending[d_idx];
    if (chunk > 512)
      chunk = 512;

    pulseChunk(2, M2_STEP_PIN, M2_DIR_PIN,
#ifdef M2_EN_PIN
               M2_EN_PIN,
#else
               -1,
#endif
               d_idx /*0=CW,1=CCW*/, chunk, halfUs);

    R.pending[d_idx] -= chunk;

    while (xQueueReceive(motorQ2, &cmd, 0) == pdTRUE)
      applyCmd_M2(R, cmd);
  }
}

// ============ Home Search function ============
static inline void stepOnce(uint8_t stepPin, uint32_t halfUs)
{
  digitalWrite(stepPin, HIGH);
  ets_delay_us(halfUs);
  digitalWrite(stepPin, LOW);
  ets_delay_us(halfUs);
}

static void HomeSearch()
{
  const uint32_t HALF_US_100pps = 5000; // 100 pps => half period 5000us

  // ---------- M1 ----------
  pinMode(M1_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  digitalWrite(M1_DIR_PIN, HIGH);
  while (digitalRead(M1_LIMIT_SWITCH_PIN) == LOW)
  {
    for (uint32_t i = 0; i < 40; ++i)
      stepOnce(M1_STEP_PIN, HALF_US_100pps);
    yield();
  }

  digitalWrite(M1_DIR_PIN, LOW);
  while (digitalRead(M1_LIMIT_SWITCH_PIN) == HIGH)
  {
    stepOnce(M1_STEP_PIN, HALF_US_100pps);
    yield();
  }
  Serial.printf("[Node%u] M1 Home Position Found!\n", NODE_ID);

  // After homing M1, reset logical position and home state
  g_M1_pos_steps = 0;
  g_M1_home_latched = false;
  g_M1_home_prev = (digitalRead(M1_LIMIT_SWITCH_PIN) == LOW);

  // ---------- M2 ----------
  digitalWrite(M2_DIR_PIN, HIGH);
  while (isM2Active())
  {
    for (uint32_t i = 0; i < 40; ++i)
      stepOnce(M2_STEP_PIN, HALF_US_100pps);
    yield();
  }

  for (uint32_t i = 0; i < 40; ++i)
    stepOnce(M2_STEP_PIN, HALF_US_100pps);

  digitalWrite(M2_DIR_PIN, LOW);
  while (!isM2Active())
  {
    stepOnce(M2_STEP_PIN, HALF_US_100pps);
    yield();
  }
  Serial.printf("[Node%u] M2 Home Position Found! (analog)\n", NODE_ID);

  // Optional: reset M2 logical position
  g_M2_pos_steps = 0;
}

// ---------- setup / loop ----------
void setup()
{
  Serial.begin(115200);
  delay(200);

  // Init GPIOs
  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
#ifdef M1_EN_PIN
  pinMode(M1_EN_PIN, OUTPUT);
  digitalWrite(M1_EN_PIN, HIGH);
#endif
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
#ifdef M2_EN_PIN
  pinMode(M2_EN_PIN, OUTPUT);
  digitalWrite(M2_EN_PIN, HIGH);
#endif
  pinMode(M1_LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // ====== ADC config for M2 analog limit ======
  analogReadResolution(12); // 12 bits
  analogSetPinAttenuation(M2_LIMIT_SWITCH_PIN, ADC_11db);
  pinMode(M2_LIMIT_SWITCH_PIN, INPUT);

  // Init logical positions & home flags
  g_M1_pos_steps = 0;
  g_M1_home_latched = false;
  g_M1_home_prev = (digitalRead(M1_LIMIT_SWITCH_PIN) == LOW);
  g_M2_pos_steps = 0;

  // Perform home search for M1 & M2
  Serial.printf("[Node%u] Performing Home Search for M1...\n", NODE_ID);
  HomeSearch();

  Serial.printf("\n=== CroMotion Node%u (per-node LED) ===\n", NODE_ID);
  Serial.printf("[Node%u] sizeof(MotionFrame)=%u bytes\n", NODE_ID, (unsigned)sizeof(cro::MotionFrame));

  // ====== ESP-NOW init ======
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[Node] ESP-NOW init failed!");
    while (true)
      delay(1000);
  }

  // Queues
  rxQ = xQueueCreate(10, sizeof(FrameWrap));
  motorQ1 = xQueueCreate(16, sizeof(MotorCmd));
  motorQ2 = xQueueCreate(16, sizeof(MotorCmd));

  esp_now_register_recv_cb(onDataRecv);

  // Tasks
  xTaskCreatePinnedToCore(taskRxProcess, "taskRxProcess", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskHeartbeat, "taskHeartbeat", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskMotor1, "taskMotor1", 4096, NULL, 2, &hMotor1, 1);
  xTaskCreatePinnedToCore(taskMotor2, "taskMotor2", 4096, NULL, 2, &hMotor2, 1);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("[Node%u] MAC=%02X:%02X:%02X:%02X:%02X:%02X  CH=%u\n",
                NODE_ID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ESPNOW_CH);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(100)); }
// =============================== end of file ===============================
