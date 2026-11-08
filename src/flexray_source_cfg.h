// PiFlex-FW - host-configurable generator schedule + params (host-testable, no
// Pico SDK). SPDX-License-Identifier: GPL-3.0-or-later
//
// Holds the RAM source schedule (the slots the generator emits) and the generator
// timing/run-state, seeded at boot with the phase-5 sample defaults and settable
// at runtime from SavvyCAN via bulk-OUT ops 0x94/0x95/0x96 (protocol.h). The
// op-byte framing is done by fr_inject_consume (flexray_injector.c, the single
// EP 0x03 framer); it calls the fr_source_cfg_apply_* parsers here once it has a
// complete op. Deliberately free of any hardware/SDK include so the whole control
// plane is unit-testable on a host (see test/hostcfg_test.c); the generator TX
// path (flexray_source_tx.c, -DFR_SOURCE only) reads the live table + params.
//
// These ops are accepted and stored in EVERY build for a uniform host contract,
// but only have a bus effect in an -DFR_SOURCE build (where the generator runs).
#ifndef PIFLEX_FLEXRAY_SOURCE_CFG_H
#define PIFLEX_FLEXRAY_SOURCE_CFG_H

#include <stdint.h>
#include <stdbool.h>
#include "flexray_source_schedule.h"   // fr_source_slot_t, FR_SRC_MAX_SLOTS

// Boot defaults for the generator timing / run-state (SDK-free; config.h's bench
// notes describe them). The generator runs by default so the standalone source
// build behaves exactly as phase 5 until the host sends 0x96.
#define FR_SRC_DEFAULT_PERIOD_US     500u
#define FR_SRC_DEFAULT_CYCLE_GAP_US  0u
#define FR_SRC_DEFAULT_ENABLE        true

// Re-seed the schedule with the boot defaults (FR_SOURCE_DEFAULT_SCHEDULE) and the
// default timing/run-state. Called from fr_inject_reset() at start-up.
void fr_source_cfg_reset(void);

// op 0x94 SET_SOURCE_SLOT: add or replace (keyed by slot id) one schedule slot
// from a complete message buffer [0x94][slot lo][slot hi][chan][N][flags][N bytes]
// (len = 6 + N). An invalid payload_len (odd, 0, or > FR_MAX_PAYLOAD) or a full
// table drops the add. msg/len are the framed op as delivered by fr_inject_consume.
void fr_source_cfg_apply_slot(const uint8_t *msg, int len);

// op 0x95 CLEAR_SOURCE_SCHEDULE: remove every schedule slot.
void fr_source_cfg_clear_schedule(void);

// op 0x96 SET_SOURCE_PARAMS: set period_us / cycle_gap_us / enable from a complete
// 10-byte message [0x96][period u32 LE][gap u32 LE][enable].
void fr_source_cfg_apply_params(const uint8_t *msg, int len);

// Live schedule accessors (used by the generator TX path and the 0xA3 status).
int                      fr_source_cfg_slot_count(void);     // 0..FR_SRC_MAX_SLOTS
const fr_source_slot_t  *fr_source_cfg_slot(int i);          // NULL if out of range
uint32_t                 fr_source_cfg_period_us(void);
uint32_t                 fr_source_cfg_cycle_gap_us(void);

// The stored enable flag (op 0x96), independent of the build.
bool                     fr_source_cfg_enabled(void);

// Count of op-0x94 SET_SOURCE_SLOT ops rejected (empty / odd / over-long payload,
// or a full table) since boot or the last schedule clear (0x95). Reported as
// source_slot_drops in the 0xA3 status so the host can see why an "Apply schedule"
// had no effect instead of a silent drop.
uint32_t                 fr_source_cfg_slot_drops(void);

// Whether the generator is ACTUALLY emitting: true only in an -DFR_SOURCE build
// with the enable flag set. Reported as source_running in the 0xA3 status, so a
// non-source build truthfully reports 0 even after a 0x96 enable.
bool                     fr_source_cfg_running(void);

#endif // PIFLEX_FLEXRAY_SOURCE_CFG_H
