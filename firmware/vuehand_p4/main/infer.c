#include "infer.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "model_weights.h"
#include "seed_poses.h"

#define FNV_OFFSET 2166136261u
#define FNV_PRIME 16777619u

static uint32_t fnv1a32(const char *s, size_t n)
{
    uint32_t h = FNV_OFFSET;
    for (size_t i = 0; i < n; i++) {
        h ^= (uint8_t)s[i];
        h *= FNV_PRIME;
    }
    return h;
}

void vh_featurize(const char *text, float *out64)
{
    for (int i = 0; i < VH_FEATURE_DIM; i++) {
        out64[i] = 0.0f;
    }
    char tok[48];
    int ti = 0;
    int any = 0;
    for (const char *p = text;; p++) {
        char c = *p;
        char lc = (char)tolower((unsigned char)c);
        bool alnum = (lc >= 'a' && lc <= 'z') || (lc >= '0' && lc <= '9');
        if (alnum && ti < (int)sizeof(tok) - 1) {
            tok[ti++] = lc;
            continue;
        }
        if (ti > 0) {
            tok[ti] = 0;
            uint32_t h = fnv1a32(tok, (size_t)ti);
            out64[h % VH_FEATURE_DIM] += 1.0f;
            out64[(h >> 8) % VH_FEATURE_DIM] -= 0.5f;
            any = 1;
            ti = 0;
        }
        if (!c) {
            break;
        }
    }
    if (!any) {
        return;
    }
    float n2 = 0.0f;
    for (int i = 0; i < VH_FEATURE_DIM; i++) {
        n2 += out64[i] * out64[i];
    }
    float n = sqrtf(n2);
    if (n > 0.0f) {
        for (int i = 0; i < VH_FEATURE_DIM; i++) {
            out64[i] /= n;
        }
    }
}

void vh_mlp(const float *x81, float *y136)
{
    float h[VH_HIDDEN];
    for (int i = 0; i < VH_HIDDEN; i++) {
        float s = VH_B1[i];
        const float *w = &VH_W1[i * VH_IN_DIM];
        for (int j = 0; j < VH_IN_DIM; j++) {
            s += w[j] * x81[j];
        }
        h[i] = s > 0.0f ? s : 0.0f;
    }
    for (int i = 0; i < VH_OUT_DIM; i++) {
        float s = VH_B2[i];
        const float *w = &VH_W2[i * VH_HIDDEN];
        for (int j = 0; j < VH_HIDDEN; j++) {
            s += w[j] * h[j];
        }
        y136[i] = s;
    }
}

void vh_denorm_frames(const float *y136, float frames[][VH_N_JOINTS])
{
    for (int k = 0; k < VH_N_KEYFRAMES; k++) {
        for (int j = 0; j < VH_N_JOINTS; j++) {
            float u = y136[k * VH_N_JOINTS + j];
            if (u < 0.0f) u = 0.0f;
            if (u > 1.0f) u = 1.0f;
            frames[k][j] = VH_ROM_LO[j] + u * (VH_ROM_HI[j] - VH_ROM_LO[j]);
        }
    }
}

static bool same_cmd(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = (char)tolower((unsigned char)*a);
        char cb = (char)tolower((unsigned char)*b);
        if (ca == '_') ca = ' ';
        if (cb == '_') cb = ' ';
        if (ca != cb) {
            return false;
        }
        a++;
        b++;
    }
    return *a == 0 && *b == 0;
}

bool vh_lookup_seed(const char *text, float frames[][VH_N_JOINTS], int *n_frames, int *duration_ms)
{
    for (int m = 0; m < VH_N_SEED; m++) {
        for (int a = 0; a < VH_SEED[m].n_aliases; a++) {
            if (!same_cmd(text, VH_SEED[m].aliases[a])) {
                continue;
            }
            int n = VH_SEED[m].n_frames;
            if (n > VH_N_KEYFRAMES) {
                n = VH_N_KEYFRAMES;
            }
            for (int i = 0; i < n; i++) {
                memcpy(frames[i], VH_SEED[m].frames[i], sizeof(float) * VH_N_JOINTS);
            }
            *n_frames = n;
            *duration_ms = VH_SEED[m].duration_ms;
            return true;
        }
    }
    return false;
}

void vh_say(const char *text, const float *current17, float frames[][VH_N_JOINTS], int *n_frames, int *duration_ms)
{
    if (vh_lookup_seed(text, frames, n_frames, duration_ms)) {
        return;
    }
    float x[VH_IN_DIM];
    float feat[VH_FEATURE_DIM];
    vh_featurize(text, feat);
    memcpy(x, feat, sizeof feat);
    for (int j = 0; j < VH_N_JOINTS; j++) {
        float span = VH_ROM_HI[j] - VH_ROM_LO[j];
        float v = current17 ? current17[j] : VH_ROM_LO[j];
        x[VH_FEATURE_DIM + j] = span > 0 ? (v - VH_ROM_LO[j]) / span : 0.0f;
    }
    float y[VH_OUT_DIM];
    vh_mlp(x, y);
    vh_denorm_frames(y, frames);
    *n_frames = VH_N_KEYFRAMES;
    *duration_ms = 600;
}
