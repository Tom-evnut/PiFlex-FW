// PiFlex-FW - runtime data-path mode manager (phase 7, host-testable, no Pico SDK).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// One firmware compiles every TX path in (bridge / inject / source) and switches
// between them, plus a passive sniff mode, at runtime from the host (control
// requests 0xA4/0xA5, see protocol.h). Only one TX mode drives the bus at a time.
//
// This core holds just the active-mode bookkeeping and the SAFE transition
// ordering; it is deliberately free of any hardware/SDK include so the sequencing
// can be unit-tested on a host (see test/mode_test.c). The hardware specifics -
// which PIO program/SMs/DMA each mode claims, and releasing the TXEN lines - live
// behind the hook callbacks (flexray_mode_hw.c provides the real ones).
//
// SAFE TRANSITION (the contract the hooks must honour): on every change the bus is
// made safe FIRST (both TXEN released = transceivers in RX, PIO TX stopped), THEN
// the old mode's PIO/SMs/DMA are torn down, THEN the new mode is entered. So TXEN
// is always released before pio1 is reprogrammed - the swap never glitches the bus.
#ifndef PIFLEX_FLEXRAY_MODE_H
#define PIFLEX_FLEXRAY_MODE_H

#include <stdbool.h>
#include "protocol.h"   // enum piflex_mode, PIFLEX_MODE_*

typedef enum piflex_mode fr_mode_t;

// Hardware actions the mode manager drives, in this order, on each transition.
//   safe_bus : stop any pio1 TX state machines and release BOTH TXEN lines
//              (inactive HIGH). Mode-independent; safe to call when nothing runs.
//   teardown : free the given mode's pio1 program / SMs / DMA (SNIFF: nothing).
//   enter    : run the given mode's *_init exactly as the dedicated build did
//              (SNIFF: nothing - RX only).
typedef struct {
    void (*safe_bus)(void);
    void (*teardown)(fr_mode_t mode);
    void (*enter)(fr_mode_t mode);
} fr_mode_hooks_t;

// Install the hooks and bring up the initial mode: safe_bus() then enter(initial).
// (No teardown - there is no prior mode at boot.) After this fr_mode_active() is
// `initial`.
void fr_mode_init(const fr_mode_hooks_t *hooks, fr_mode_t initial);

// Request a mode. An out-of-range value is ignored (returns false, no transition).
// The same mode as the active one is a no-op (returns true, no hook calls). A real
// change runs safe_bus() -> teardown(old) -> enter(new) and returns true. Call
// only from the main loop / control callback, never an ISR; safe to call again.
bool fr_mode_set(unsigned requested);

// The mode currently driving the data path.
fr_mode_t fr_mode_active(void);

#endif // PIFLEX_FLEXRAY_MODE_H
