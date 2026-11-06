// PiFlex-FW - FlexRay injector control plane (host-testable, no Pico SDK).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Parses the SavvyCAN bulk-OUT injector protocol (ops 0x90/0x91, see
// protocol.h), verifies each override against the compiled rule table
// (flexray_injector_rules.h) and an AUTOSAR E2E CRC-8, and holds the pending
// replacement bytes for the TX path to splice. Deliberately free of any
// hardware/SDK include so the whole control plane can be unit-tested on a host
// (see test/injector_test.c); main.c does the USB reads and feeds the bytes in,
// and the TX path queries fr_inject_match() as it forwards each frame.
//
// SAFETY: the injector defaults OFF. fr_inject_reset() clears every pending
// override and disables injection; main.c calls it at start-up so the bridge
// never injects until the host explicitly enables it (op 0x91 [enabled=1]) AND
// has submitted a matching override. SavvyCAN pushes the enable state on every
// (re)connect (applyInjectorEnable), so an OFF default is corrected by the host
// whenever a bus is taken out of listen-only - OFF is strictly the safe default.
#ifndef PIFLEX_FLEXRAY_INJECTOR_H
#define PIFLEX_FLEXRAY_INJECTOR_H

#include <stdint.h>
#include <stdbool.h>

// AUTOSAR E2E Profile CRC-8 (polynomial 0x1D, MSB-first, no final XOR). Matches
// SavvyCAN's PicoFlexRay::e2eCrc8. The host submits each override slice prefixed
// with e2eCrc8(payload, init 0xF1); the firmware recomputes and rejects a
// mismatch. Exposed for reuse and tests.
#define FR_INJ_E2E_INIT  0xF1u
uint8_t fr_e2e_crc8(const uint8_t *data, int len, uint8_t init);

// Reset all injector state: disable injection and drop every pending override.
void fr_inject_reset(void);

// Global injector enable (op 0x91). Disabling leaves pending overrides stored
// but stops the TX path injecting (fr_inject_match returns no match while off).
void fr_inject_set_enabled(bool enabled);
bool fr_inject_enabled(void);

// Try to parse and handle ONE injector message at the front of buf (len bytes
// available). Returns:
//   > 0  bytes consumed - one complete message was parsed (and, for 0x90,
//        verified and either stored or dropped; 0x90/0x91 send no host ACK).
//   = 0  incomplete - a known op is present but more bytes are needed; call
//        again once more have arrived.
//   < 0  the leading byte is not a known op - the caller should resync (drop).
// Pure function over its inputs + injector state; no I/O, so it is host-testable.
int fr_inject_consume(const uint8_t *buf, int len);

// TX-path query. If injection is enabled and a pending override matches this
// forwarded frame's slot ID and cycle, returns true and sets *out_offset /
// *out_len to the replace window and *out_data to the stored replacement bytes
// (out_len bytes). Returns false (no splice) otherwise. The TX path must still
// check the window fits the frame's payload before splicing.
bool fr_inject_match(uint16_t slot_id, uint8_t cycle,
                     uint8_t *out_offset, uint8_t *out_len,
                     const uint8_t **out_data);

#endif // PIFLEX_FLEXRAY_INJECTOR_H
