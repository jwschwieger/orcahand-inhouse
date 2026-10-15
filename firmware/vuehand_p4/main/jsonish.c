#include "jsonish.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *find_key(const char *json, const char *key)
{
    char pat[64];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(json, pat);
    if (!p) {
        return NULL;
    }
    p += strlen(pat);
    while (*p && (*p == ' ' || *p == '\t' || *p == ':')) {
        p++;
    }
    return p;
}

bool json_get_str(const char *json, const char *key, char *out, size_t n)
{
    const char *p = find_key(json, key);
    if (!p || *p != '"') {
        return false;
    }
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < n) {
        if (*p == '\\' && p[1]) {
            p++;
        }
        out[i++] = *p++;
    }
    out[i] = 0;
    return true;
}

bool json_get_int(const char *json, const char *key, int *out)
{
    const char *p = find_key(json, key);
    if (!p) {
        return false;
    }
    *out = (int)strtol(p, NULL, 10);
    return true;
}

bool json_get_bool(const char *json, const char *key, bool *out)
{
    const char *p = find_key(json, key);
    if (!p) {
        return false;
    }
    if (strncmp(p, "true", 4) == 0) {
        *out = true;
        return true;
    }
    if (strncmp(p, "false", 5) == 0) {
        *out = false;
        return true;
    }
    return false;
}

int json_get_vec(const char *json, const char *key, float *out, int maxn)
{
    const char *p = find_key(json, key);
    if (!p || *p != '[') {
        return 0;
    }
    p++;
    int n = 0;
    while (*p && *p != ']' && n < maxn) {
        while (*p && (isspace((unsigned char)*p) || *p == ',')) {
            p++;
        }
        if (*p == ']') {
            break;
        }
        char *end = NULL;
        out[n++] = strtof(p, &end);
        if (end == p) {
            break;
        }
        p = end;
    }
    return n;
}
