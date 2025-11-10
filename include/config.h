#ifndef CONFIG_H
#define CONFIG_H
// -------------------- Project Configuration --------------------

// -------------------- Pattern table (19 timeframes) --------------------
// Each string: 19 O/X/F characters (O=ON, X=Toggle, F=OFF)
// M1 not supported X for toggle
// -----------------------------------------------------------------------
struct MotorPreset
{
    uint8_t speed; // abstract speed (0-255 or your custom scale)
    uint16_t degree; // degrees per command
    uint8_t times; // repeat count per sequence
};
static MotorPreset M1_PRESET{100, 40, 1};
static MotorPreset M2_PRESET{100, 360, 1};


// -------------------------- Pattern 1 --------------------------
// LED
static const char *P1_LED[6] = {
    "O O O F F X F O O O F O F O F F O O F", // Node1
    "F O O O F F X F O O F F O F O F O F F", // Node2
    "F F O O O F F X F O O F F O F O F O O", // Node3
    "F F F O O O F F X F O O F F O F O F O", // Node4
    "F F F F O O O F F X F O O F F O F O F", // Node5
    "F F F F F O O O F F X F O O F F O F O"  // Node6
};
// M1
static const char *P1_M1[6] = {
    "F F O O F F F O O F F O F O F F O O F", // Node1
    "F F F O O F F F O O F F F O F O F O F", // Node2
    "F F F F O O F F F O O F F F O F O F F", // Node3
    "F F F F F F O O F F F O O F F F O F O", // Node4
    "F F F F O O F F F O O F F F O F O F F", // Node5
    "F F O O O F F F O O F F F O F O F F F"  // Node6
};
// M2
static const char *P1_M2[6] = {
    "F X F F F F F X X F F X F X F F X X F", // Node1
    "F X F F F F F X X F F X F X F F X X F", // Node2
    "F X F F F F F X X F F X F X F F X X F", // Node3
    "F X F F F F F X X F F X F X F F X X F", // Node4
    "F X F F F F F X X F F X F X F F X X F", // Node5
    "F X F F F F F X X F F X F X F F X X F"  // Node6
};

// -------------------------- Pattern 2 --------------------------
// LED
static const char *P2_LED[6] = {
    "O O O O O X O O O O O O O O O O O O O", // Node1
    "O O O O O O X F O O F F O F O F O F F", // Node2
    "F F O O O F F X F O O F F O F O F O O", // Node3
    "F F F O O O F F X F O O F F O F O F O", // Node4
    "F F F F O O O F F X F O O F F O F O F", // Node5
    "F F F F F O O O F F X F O O F F O F O"  // Node6
};
// M1
static const char *P2_M1[6] = {
    "F F O O F F F O O F F O F O F F O O F", // Node1
    "F F F O O F F F O O F F F O F O F O F", // Node2
    "F F F F O O F F F O O F F F O F O F F", // Node3
    "F F F F F F O O F F F O O F F F O F O", // Node4
    "F F F F O O F F F O O F F F O F O F F", // Node5
    "F F O O O F F F O O F F F O F O F F F"  // Node6
};
// M2
static const char *P2_M2[6] = {
    "F X F F F F F X X F F X F X F F X X F", // Node1
    "F X F F F F F X X F F X F X F F X X F", // Node2
    "F X F F F F F X X F F X F X F F X X F", // Node3
    "F X F F F F F X X F F X F X F F X X F", // Node4
    "F X F F F F F X X F F X F X F F X X F", // Node5
    "F X F F F F F X X F F X F X F F X X F"  // Node6
};

extern uint8_t sequence_selection; // 1 or 2

// LIGHT SENSOR CONFIG
#define LIGHT_SENSOR_THRESHOLD_LUX_MIN 50.0f // very dark
#define LIGHT_SENSOR_THRESHOLD_LUX_MID 300.0f // weak light
#define LIGHT_SENSOR_THRESHOLD_LUX_MAX 1000.0f // strong light

// NUMBER OF SEQUENCES PER PATTERN
#define PATTERN_NUM_SEQUENCES 19

// Time interval (ms) per sequence/frame
#define PATTERN_TIMEFRAME_MS 3000

#endif // CONFIG_H