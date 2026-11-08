// PiFlex-FW - runtime data-path mode manager. SDK-free so the transition ordering
// is unit-testable on a host (see test/mode_test.c). Clean-room: our own design,
// not derived from any third-party firmware.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_mode.h"

static const fr_mode_hooks_t *s_hooks  = 0;
static fr_mode_t              s_active  = PIFLEX_MODE_SNIFF;

void fr_mode_init(const fr_mode_hooks_t *hooks, fr_mode_t initial) {
    s_hooks = hooks;
    // Bring the bus to a safe state, then start the initial mode. There is no
    // previous mode to tear down at boot.
    s_hooks->safe_bus();
    s_hooks->enter(initial);
    s_active = initial;
}

bool fr_mode_set(unsigned requested) {
    if (requested > (unsigned)PIFLEX_MODE_MAX) return false;   // ignore out-of-range
    const fr_mode_t next = (fr_mode_t)requested;
    if (next == s_active) return true;                         // no-op

    // SAFE ORDER: release TXEN + stop PIO TX, then free the old mode's resources,
    // then enter the new mode. TXEN is released before pio1 is reprogrammed.
    s_hooks->safe_bus();
    s_hooks->teardown(s_active);
    s_hooks->enter(next);
    s_active = next;
    return true;
}

fr_mode_t fr_mode_active(void) { return s_active; }
