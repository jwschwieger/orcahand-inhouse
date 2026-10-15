/* Auto-generated official ORCA v2 joint tables — do not edit. */
#ifndef VUEHAND_JOINTS_TABLE_H
#define VUEHAND_JOINTS_TABLE_H

#define VH_N_JOINTS 17
#ifndef VH_N_KEYFRAMES
#define VH_N_KEYFRAMES 8
#endif
#define VH_N_MOTORS 17
#define VH_WRIST_MOTOR_ID 1
#define VH_BAUDRATE 1000000
#define VH_MAX_CURRENT_MA 300
#define VH_RATIO 0.007f
#define VH_TICKS_PER_REV 4096

static const char *VH_JOINT_NAMES[VH_N_JOINTS] = { "wrist", "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip", "index_abd", "index_mcp", "index_pip", "middle_abd", "middle_mcp", "middle_pip", "ring_abd", "ring_mcp", "ring_pip", "pinky_abd", "pinky_mcp", "pinky_pip" };
static const float VH_ROM_LO[VH_N_JOINTS] = { -65.000f, -45.000f, -18.000f, -25.000f, -15.000f, -30.000f, -25.000f, -15.000f, -27.000f, -25.000f, -15.000f, -27.000f, -25.000f, -15.000f, -30.000f, -25.000f, -15.000f };
static const float VH_ROM_HI[VH_N_JOINTS] = { 35.000f, 33.000f, 55.000f, 100.000f, 107.000f, 25.000f, 100.000f, 107.000f, 27.000f, 100.000f, 107.000f, 27.000f, 100.000f, 107.000f, 30.000f, 100.000f, 107.000f };
static const int VH_RIGHT_MAP[VH_N_JOINTS] = { -1, 17, 14, 15, 16, 4, 3, 2, 5, 10, 9, 6, -7, 8, -13, 12, -11 };
static const int VH_LEFT_MAP[VH_N_JOINTS] = { -1, -10, -13, 12, 11, -7, -8, -9, -6, -17, -2, -5, 4, -3, 14, -15, 16 };
static const int VH_MOTOR_IDS[VH_N_MOTORS] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 };

#endif
