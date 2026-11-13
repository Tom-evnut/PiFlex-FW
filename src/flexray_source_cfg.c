// PiFlex-FW - host-configurable generator schedule + params. SDK-free so the RAM
// tables and the 0x94/0x95/0x96 parsing can be unit-tested on a host (see
// test/hostcfg_test.c). Clean-room: our own SavvyCAN interop contract (protocol.h),
// not derived from any third-party firmware.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_source_cfg.h"
#include "flexray_source_schedule.h"
#include "flexray_decode.h"   // FR_MAX_PAYLOAD
#include "protocol.h"
#include <string.h>

static fr_source_slot_t s_sched[FR_SRC_MAX_SLOTS];
static int              s_slot_count = 0;

static uint32_t s_period_us = FR_SRC_DEFAULT_PERIOD_US;
static uint32_t s_gap_us    = FR_SRC_DEFAULT_CYCLE_GAP_US;
static bool     s_enable    = FR_SRC_DEFAULT_ENABLE;

static uint32_t s_slot_drops = 0;   // 0x94 adds rejected (full / bad payload_len)
static bool     s_active     = false;  // true only while SOURCE mode drives the bus

void fr_source_cfg_reset(void) {
    memset(s_sched, 0, sizeof s_sched);
    s_slot_count = 0;
    for (int i = 0; i < FR_SRC_DEFAULT_SLOT_COUNT && s_slot_count < FR_SRC_MAX_SLOTS; i++)
        s_sched[s_slot_count++] = FR_SOURCE_DEFAULT_SCHEDULE[i];

    s_period_us  = FR_SRC_DEFAULT_PERIOD_US;
    s_gap_us     = FR_SRC_DEFAULT_CYCLE_GAP_US;
    s_enable     = FR_SRC_DEFAULT_ENABLE;
    s_slot_drops = 0;
    s_active     = false;
}

void fr_source_cfg_set_active(bool active) { s_active = active; }

static int find_slot(uint16_t slot_id) {
    for (int i = 0; i < s_slot_count; i++)
        if (s_sched[i].slot_id == slot_id) return i;
    return -1;
}

void fr_source_cfg_apply_slot(const uint8_t *msg, int len) {
    if (len < 6) return;                                   // framer guarantees this
    const uint16_t slot_id  = (uint16_t)((msg[1] | (msg[2] << 8)) & 0x7FFu);
    const uint8_t  channels = (uint8_t)(msg[3] & (FR_SRC_CH_FR1 | FR_SRC_CH_FR2));
    const int      n        = msg[4];
    const uint8_t  flags    = msg[5];

    // A FlexRay payload is a whole number of 2-byte words and bounded by the
    // decoder maximum. Reject anything else (don't store a frame we can't build).
    if (n == 0 || (n & 1) || n > FR_MAX_PAYLOAD || 6 + n > len) { s_slot_drops++; return; }

    int i = find_slot(slot_id);
    if (i < 0) {
        if (s_slot_count >= FR_SRC_MAX_SLOTS) { s_slot_drops++; return; }   // full -> drop
        i = s_slot_count++;
        memset(&s_sched[i], 0, sizeof s_sched[i]);
    }
    s_sched[i].slot_id     = slot_id;
    s_sched[i].payload_len = (uint8_t)n;
    s_sched[i].channels    = channels;
    s_sched[i].flags       = flags;
    memcpy(s_sched[i].payload, &msg[6], (size_t)n);
}

void fr_source_cfg_clear_schedule(void) {
    memset(s_sched, 0, sizeof s_sched);
    s_slot_count = 0;
    s_slot_drops = 0;   // drop count is "since boot or the last schedule clear"
}

void fr_source_cfg_apply_params(const uint8_t *msg, int len) {
    if (len < 10) return;                                  // framer guarantees this
    s_period_us = (uint32_t)msg[1] | ((uint32_t)msg[2] << 8) |
                  ((uint32_t)msg[3] << 16) | ((uint32_t)msg[4] << 24);
    s_gap_us    = (uint32_t)msg[5] | ((uint32_t)msg[6] << 8) |
                  ((uint32_t)msg[7] << 16) | ((uint32_t)msg[8] << 24);
    s_enable    = (msg[9] != 0);
}

int                     fr_source_cfg_slot_count(void)  { return s_slot_count; }
uint32_t                fr_source_cfg_period_us(void)    { return s_period_us; }
uint32_t                fr_source_cfg_cycle_gap_us(void) { return s_gap_us; }
bool                    fr_source_cfg_enabled(void)      { return s_enable; }
uint32_t                fr_source_cfg_slot_drops(void)   { return s_slot_drops; }

const fr_source_slot_t *fr_source_cfg_slot(int i) {
    if (i < 0 || i >= s_slot_count) return NULL;
    return &s_sched[i];
}

bool fr_source_cfg_running(void) {
    // Truthful "generator actually emitting": SOURCE mode is the active data path
    // (set by the mode manager), the host has it enabled, and there is a slot to
    // emit. In any other mode s_active is false, so this reports 0.
    return s_active && s_enable && s_slot_count > 0;
}
