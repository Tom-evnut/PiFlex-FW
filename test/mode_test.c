// PiFlex-FW - host test for the runtime data-path mode manager (flexray_mode.c).
// Exercises the SDK-free transition bookkeeping with fake hooks that record the
// call order, so the SAFE transition contract is pinned without a device:
//   - valid vs invalid (out-of-range) mode requests,
//   - no-op when the requested mode is already active,
//   - the exact order safe_bus -> teardown(old) -> enter(new) on every change,
//     i.e. both TXEN released (safe_bus) BEFORE the new mode reprograms pio1.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src mode_test.c ../src/flexray_mode.c -o mode_test && ./mode_test
#include "flexray_mode.h"
#include "protocol.h"
#include <stdio.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// Recorded hook calls.
typedef enum { OP_SAFE, OP_TEARDOWN, OP_ENTER } op_t;
typedef struct { op_t op; int mode; } event_t;

static event_t s_log[64];
static int     s_n = 0;

static void log_reset(void)                 { s_n = 0; }
static void push(op_t op, int mode)         { if (s_n < 64) { s_log[s_n].op = op; s_log[s_n].mode = mode; s_n++; } }

static void fake_safe_bus(void)             { push(OP_SAFE, -1); }
static void fake_teardown(fr_mode_t mode)   { push(OP_TEARDOWN, (int)mode); }
static void fake_enter(fr_mode_t mode)      { push(OP_ENTER, (int)mode); }

static const fr_mode_hooks_t FAKE = {
    .safe_bus = fake_safe_bus,
    .teardown = fake_teardown,
    .enter    = fake_enter,
};

// Assert the last three events are a full safe transition old -> new, in order.
static void expect_transition(int old_mode, int new_mode, const char *what) {
    CHECK(s_n >= 3, what);
    if (s_n < 3) return;
    const event_t *a = &s_log[s_n - 3], *b = &s_log[s_n - 2], *c = &s_log[s_n - 1];
    CHECK(a->op == OP_SAFE, "transition releases the bus (safe_bus) first");
    CHECK(b->op == OP_TEARDOWN && b->mode == old_mode, "then tears down the old mode");
    CHECK(c->op == OP_ENTER && c->mode == new_mode, "then enters the new mode");
}

static void test_init_default_sniff(void) {
    log_reset();
    fr_mode_init(&FAKE, PIFLEX_MODE_SNIFF);
    CHECK(fr_mode_active() == PIFLEX_MODE_SNIFF, "boot mode is SNIFF");
    CHECK(s_n == 2 && s_log[0].op == OP_SAFE && s_log[1].op == OP_ENTER
              && s_log[1].mode == PIFLEX_MODE_SNIFF,
          "boot = safe_bus then enter(SNIFF), no teardown");
}

static void test_transitions(void) {
    fr_mode_init(&FAKE, PIFLEX_MODE_SNIFF);

    log_reset();
    CHECK(fr_mode_set(PIFLEX_MODE_BRIDGE), "SET BRIDGE accepted");
    CHECK(fr_mode_active() == PIFLEX_MODE_BRIDGE, "active = BRIDGE");
    expect_transition(PIFLEX_MODE_SNIFF, PIFLEX_MODE_BRIDGE, "SNIFF->BRIDGE is a full safe transition");

    log_reset();
    CHECK(fr_mode_set(PIFLEX_MODE_INJECT), "SET INJECT accepted");
    expect_transition(PIFLEX_MODE_BRIDGE, PIFLEX_MODE_INJECT, "BRIDGE->INJECT is a full safe transition");

    log_reset();
    CHECK(fr_mode_set(PIFLEX_MODE_SOURCE), "SET SOURCE accepted");
    expect_transition(PIFLEX_MODE_INJECT, PIFLEX_MODE_SOURCE, "INJECT->SOURCE is a full safe transition");

    log_reset();
    CHECK(fr_mode_set(PIFLEX_MODE_SNIFF), "SET SNIFF accepted");
    expect_transition(PIFLEX_MODE_SOURCE, PIFLEX_MODE_SNIFF, "SOURCE->SNIFF is a full safe transition");
}

static void test_noop_same_mode(void) {
    fr_mode_init(&FAKE, PIFLEX_MODE_BRIDGE);
    log_reset();
    CHECK(fr_mode_set(PIFLEX_MODE_BRIDGE), "SET to the active mode returns true");
    CHECK(s_n == 0, "SET to the active mode does nothing (no hook calls)");
    CHECK(fr_mode_active() == PIFLEX_MODE_BRIDGE, "active unchanged by a no-op SET");
}

static void test_invalid_ignored(void) {
    fr_mode_init(&FAKE, PIFLEX_MODE_INJECT);
    log_reset();
    CHECK(!fr_mode_set(PIFLEX_MODE_MAX + 1), "out-of-range mode is rejected");
    CHECK(!fr_mode_set(255), "a wild mode value is rejected");
    CHECK(s_n == 0, "a rejected SET performs no transition");
    CHECK(fr_mode_active() == PIFLEX_MODE_INJECT, "active unchanged after a rejected SET");
}

// TXEN-released-before-reprogram: in every real transition the safe_bus event
// (which releases both TXEN) strictly precedes the enter event (which reprograms
// pio1). Pinned explicitly since it is the bus-safety guarantee.
static void test_txen_released_before_enter(void) {
    fr_mode_init(&FAKE, PIFLEX_MODE_SNIFF);
    log_reset();
    fr_mode_set(PIFLEX_MODE_SOURCE);
    int safe_i = -1, enter_i = -1, teardown_i = -1;
    for (int i = 0; i < s_n; i++) {
        if (s_log[i].op == OP_SAFE)     safe_i = i;
        if (s_log[i].op == OP_TEARDOWN) teardown_i = i;
        if (s_log[i].op == OP_ENTER)    enter_i = i;
    }
    CHECK(safe_i >= 0 && teardown_i > safe_i && enter_i > teardown_i,
          "safe_bus (TXEN released) precedes teardown precedes enter (reprogram)");
}

int main(void) {
    test_init_default_sniff();
    test_transitions();
    test_noop_same_mode();
    test_invalid_ignored();
    test_txen_released_before_enter();

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
