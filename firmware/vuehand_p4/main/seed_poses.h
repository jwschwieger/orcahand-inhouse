/* Auto-generated from data/seed_motions.json — do not edit. */
#ifndef VUEHAND_SEED_POSES_H
#define VUEHAND_SEED_POSES_H

#include "joints_table.h"

typedef struct {
    const char *name;
    const char **aliases;
    int n_aliases;
    const float (*frames)[VH_N_JOINTS];
    int n_frames;
    int duration_ms;
} vh_seed_motion_t;

static const float SEED_0_FRAMES[1][VH_N_JOINTS] = {
  { -20.0000f, 0.0000f, 50.0000f, 33.0000f, 18.0000f, -14.0000f, 2.0000f, 6.0000f, -4.0000f, 2.0000f, 4.0000f, 10.0000f, -2.0000f, 8.0000f, 22.0000f, -4.0000f, -2.0000f }
};
static const char *SEED_0_ALIASES[] = { "rest", "neutral", "home", "idle" };
static const float SEED_1_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, 10.0000f, 45.0000f, 5.0000f, 5.0000f, -12.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 10.0000f, 0.0000f, 0.0000f, 18.0000f, 0.0000f, 0.0000f }
};
static const char *SEED_1_ALIASES[] = { "open", "let go", "release", "open hand", "drop it", "ungrasp" };
static const float SEED_2_FRAMES[1][VH_N_JOINTS] = {
  { 20.0000f, 15.0000f, 50.0000f, 10.0000f, 8.0000f, -20.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 16.0000f, -5.0000f, 0.0000f, 24.0000f, -5.0000f, 0.0000f }
};
static const char *SEED_2_ALIASES[] = { "high_five", "high 5", "high five", "wave palm", "palm out" };
static const float SEED_3_FRAMES[1][VH_N_JOINTS] = {
  { -10.0000f, -10.0000f, 30.0000f, 70.0000f, 60.0000f, 0.0000f, 65.0000f, 80.0000f, 0.0000f, 75.0000f, 85.0000f, 0.0000f, 90.0000f, 95.0000f, 5.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_3_ALIASES[] = { "fist", "close", "make a fist", "clench" };
static const float SEED_4_FRAMES[1][VH_N_JOINTS] = {
  { -15.0000f, 5.0000f, 25.0000f, 55.0000f, 40.0000f, -4.0000f, 70.0000f, 55.0000f, 0.0000f, 75.0000f, 60.0000f, 4.0000f, 80.0000f, 65.0000f, 8.0000f, 82.0000f, 68.0000f }
};
static const char *SEED_4_ALIASES[] = { "power_grasp", "grasp", "grasp the corndog", "grab tool", "wrap grasp", "hold the handle", "power grip" };
static const float SEED_5_FRAMES[1][VH_N_JOINTS] = {
  { -5.0000f, 15.0000f, 35.0000f, 40.0000f, 30.0000f, -6.0000f, 45.0000f, 35.0000f, 4.0000f, 80.0000f, 85.0000f, 8.0000f, 85.0000f, 90.0000f, 12.0000f, 88.0000f, 92.0000f }
};
static const char *SEED_5_ALIASES[] = { "pinch", "precision grasp", "pick up small part", "two finger pinch", "tweezer" };
static const float SEED_6_FRAMES[1][VH_N_JOINTS] = {
  { 5.0000f, -5.0000f, 15.0000f, 55.0000f, 45.0000f, -8.0000f, -5.0000f, 0.0000f, 4.0000f, 90.0000f, 95.0000f, 8.0000f, 90.0000f, 95.0000f, 12.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_6_ALIASES[] = { "point", "index point", "indicate", "point at" };
static const float SEED_7_FRAMES[1][VH_N_JOINTS] = {
  { 10.0000f, 20.0000f, 50.0000f, 0.0000f, 0.0000f, 0.0000f, 90.0000f, 95.0000f, 0.0000f, 90.0000f, 95.0000f, 0.0000f, 90.0000f, 95.0000f, 5.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_7_ALIASES[] = { "thumbs_up", "ok", "thumbs up", "approve" };
static const float SEED_8_FRAMES[1][VH_N_JOINTS] = {
  { -60.0000f, 0.0000f, 50.0000f, 33.0000f, 18.0000f, -14.0000f, 2.0000f, 6.0000f, -4.0000f, 2.0000f, 4.0000f, 10.0000f, -2.0000f, 8.0000f, 22.0000f, -4.0000f, -2.0000f }
};
static const char *SEED_8_ALIASES[] = { "wrist_in", "pull wrist inward", "wrist flex", "curl wrist", "wrist toward forearm" };
static const float SEED_9_FRAMES[1][VH_N_JOINTS] = {
  { 30.0000f, 0.0000f, 50.0000f, 33.0000f, 18.0000f, -14.0000f, 2.0000f, 6.0000f, -4.0000f, 2.0000f, 4.0000f, 10.0000f, -2.0000f, 8.0000f, 22.0000f, -4.0000f, -2.0000f }
};
static const char *SEED_9_ALIASES[] = { "wrist_out", "extend wrist", "wrist back", "push wrist out" };
static const float SEED_10_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, -5.0000f, 15.0000f, 55.0000f, 40.0000f, -8.0000f, -5.0000f, 0.0000f, 4.0000f, 90.0000f, 95.0000f, 8.0000f, 90.0000f, 95.0000f, 12.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_10_ALIASES[] = { "count_1", "one", "count one" };
static const float SEED_11_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, -5.0000f, 15.0000f, 55.0000f, 40.0000f, -12.0000f, -5.0000f, 0.0000f, 6.0000f, -5.0000f, 0.0000f, 8.0000f, 90.0000f, 95.0000f, 12.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_11_ALIASES[] = { "count_2", "two", "peace", "count two" };
static const float SEED_12_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, -5.0000f, 15.0000f, 55.0000f, 40.0000f, -12.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 10.0000f, -5.0000f, 0.0000f, 12.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_12_ALIASES[] = { "count_3", "three", "count three" };
static const float SEED_13_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, -8.0000f, 10.0000f, 60.0000f, 50.0000f, -16.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 12.0000f, -5.0000f, 0.0000f, 20.0000f, -5.0000f, 0.0000f }
};
static const char *SEED_13_ALIASES[] = { "count_4", "four", "count four" };
static const float SEED_14_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, 15.0000f, 50.0000f, 8.0000f, 5.0000f, -18.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 14.0000f, -5.0000f, 0.0000f, 22.0000f, -5.0000f, 0.0000f }
};
static const char *SEED_14_ALIASES[] = { "count_5", "five", "count five", "open five" };
static const float SEED_15_FRAMES[6][VH_N_JOINTS] = {
  { 0.0000f, 15.0000f, 50.0000f, 8.0000f, 5.0000f, -18.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 14.0000f, -5.0000f, 0.0000f, 22.0000f, -5.0000f, 0.0000f },
  { 0.0000f, -8.0000f, 10.0000f, 60.0000f, 50.0000f, -16.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 12.0000f, -5.0000f, 0.0000f, 20.0000f, -5.0000f, 0.0000f },
  { 0.0000f, -5.0000f, 15.0000f, 55.0000f, 40.0000f, -12.0000f, -5.0000f, 0.0000f, 0.0000f, -5.0000f, 0.0000f, 10.0000f, -5.0000f, 0.0000f, 12.0000f, 90.0000f, 95.0000f },
  { 0.0000f, -5.0000f, 15.0000f, 55.0000f, 40.0000f, -12.0000f, -5.0000f, 0.0000f, 6.0000f, -5.0000f, 0.0000f, 8.0000f, 90.0000f, 95.0000f, 12.0000f, 90.0000f, 95.0000f },
  { 0.0000f, -5.0000f, 15.0000f, 55.0000f, 40.0000f, -8.0000f, -5.0000f, 0.0000f, 4.0000f, 90.0000f, 95.0000f, 8.0000f, 90.0000f, 95.0000f, 12.0000f, 90.0000f, 95.0000f },
  { -10.0000f, -10.0000f, 30.0000f, 70.0000f, 60.0000f, 0.0000f, 65.0000f, 80.0000f, 0.0000f, 75.0000f, 85.0000f, 0.0000f, 90.0000f, 95.0000f, 5.0000f, 90.0000f, 95.0000f }
};
static const char *SEED_15_ALIASES[] = { "countdown", "count down", "count down with fingers", "5 4 3 2 1" };
static const float SEED_16_FRAMES[1][VH_N_JOINTS] = {
  { 10.0000f, 5.0000f, 20.0000f, 20.0000f, 10.0000f, 0.0000f, 70.0000f, 80.0000f, 0.0000f, 75.0000f, 85.0000f, 0.0000f, 75.0000f, 85.0000f, 4.0000f, 70.0000f, 80.0000f }
};
static const char *SEED_16_ALIASES[] = { "hook", "hook grasp", "carry bag", "finger hook" };
static const float SEED_17_FRAMES[1][VH_N_JOINTS] = {
  { 0.0000f, 10.0000f, 20.0000f, 35.0000f, 25.0000f, 0.0000f, 55.0000f, 40.0000f, 0.0000f, 70.0000f, 70.0000f, 4.0000f, 75.0000f, 75.0000f, 8.0000f, 78.0000f, 78.0000f }
};
static const char *SEED_17_ALIASES[] = { "lateral_pinch", "key pinch", "hold a card", "lateral grasp" };

#define VH_N_SEED 18
static const vh_seed_motion_t VH_SEED[VH_N_SEED] = {
  { "rest", SEED_0_ALIASES, 4, SEED_0_FRAMES, 1, 500 },
  { "open", SEED_1_ALIASES, 6, SEED_1_FRAMES, 1, 450 },
  { "high_five", SEED_2_ALIASES, 5, SEED_2_FRAMES, 1, 400 },
  { "fist", SEED_3_ALIASES, 4, SEED_3_FRAMES, 1, 500 },
  { "power_grasp", SEED_4_ALIASES, 7, SEED_4_FRAMES, 1, 550 },
  { "pinch", SEED_5_ALIASES, 5, SEED_5_FRAMES, 1, 450 },
  { "point", SEED_6_ALIASES, 4, SEED_6_FRAMES, 1, 400 },
  { "thumbs_up", SEED_7_ALIASES, 4, SEED_7_FRAMES, 1, 400 },
  { "wrist_in", SEED_8_ALIASES, 5, SEED_8_FRAMES, 1, 500 },
  { "wrist_out", SEED_9_ALIASES, 4, SEED_9_FRAMES, 1, 500 },
  { "count_1", SEED_10_ALIASES, 3, SEED_10_FRAMES, 1, 300 },
  { "count_2", SEED_11_ALIASES, 4, SEED_11_FRAMES, 1, 300 },
  { "count_3", SEED_12_ALIASES, 3, SEED_12_FRAMES, 1, 300 },
  { "count_4", SEED_13_ALIASES, 3, SEED_13_FRAMES, 1, 300 },
  { "count_5", SEED_14_ALIASES, 4, SEED_14_FRAMES, 1, 300 },
  { "countdown", SEED_15_ALIASES, 4, SEED_15_FRAMES, 6, 2800 },
  { "hook", SEED_16_ALIASES, 4, SEED_16_FRAMES, 1, 450 },
  { "lateral_pinch", SEED_17_ALIASES, 4, SEED_17_FRAMES, 1, 400 }
};

#endif
