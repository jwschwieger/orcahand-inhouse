#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool json_get_str(const char *json, const char *key, char *out, size_t n);
bool json_get_int(const char *json, const char *key, int *out);
bool json_get_bool(const char *json, const char *key, bool *out);
int json_get_vec(const char *json, const char *key, float *out, int maxn);
