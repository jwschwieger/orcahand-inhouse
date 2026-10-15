#pragma once

#include <stdbool.h>

#include "joints_table.h"

void vh_featurize(const char *text, float *out64);
void vh_mlp(const float *x81, float *y136);
void vh_denorm_frames(const float *y136, float frames[][VH_N_JOINTS]);
bool vh_lookup_seed(const char *text, float frames[][VH_N_JOINTS], int *n_frames, int *duration_ms);
void vh_say(const char *text, const float *current17, float frames[][VH_N_JOINTS], int *n_frames, int *duration_ms);
