// PiFlex-FW - TinyUSB configuration.
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

// The Pico SDK's TinyUSB integration normally sets these; guard so we don't
// clash with it, but provide sane defaults for a standalone build.
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU          OPT_MCU_RP2040   // RP2350 uses the rp2 family port
#endif
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_PICO
#endif

#define CFG_TUD_ENABLED       1
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC           0
#define CFG_TUD_MSC           0
#define CFG_TUD_HID           0
#define CFG_TUD_MIDI          0
#define CFG_TUD_VENDOR        1

// Panda-style bulk pipes: small OUT (control/inject), large IN (frame stream).
#define CFG_TUD_VENDOR_RX_BUFSIZE 512
#define CFG_TUD_VENDOR_TX_BUFSIZE 4096

#endif // _TUSB_CONFIG_H_
