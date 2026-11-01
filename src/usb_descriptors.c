// PiFlex-FW - USB descriptors. Enumerates Panda-compatible (vendor class,
// VID 0x3801 / PID 0xDDCC, serial "picoflex...") so SavvyCAN's PicoFlexRay
// connection binds with no host-side changes.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "tusb.h"
#include "protocol.h"
#include "pico/unique_id.h"
#include <string.h>
#include <stdio.h>

// --- Device descriptor -----------------------------------------------------
static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xFF,   // vendor-specific (Panda)
    .bDeviceSubClass    = 0xFF,
    .bDeviceProtocol    = 0xFF,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = PIFLEX_USB_VID,
    .idProduct          = PIFLEX_USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&desc_device;
}

// --- Configuration descriptor ----------------------------------------------
enum { ITF_NUM_VENDOR = 0, ITF_NUM_TOTAL };

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    // One vendor interface with bulk OUT + bulk IN endpoints.
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, 0, PIFLEX_EP_OUT, PIFLEX_EP_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

// --- String descriptors -----------------------------------------------------
static char serial_str[32];

static const char *const string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: language (English, 0x0409)
    "comma.ai",                  // 1: manufacturer (Panda)
    "panda",                     // 2: product
    serial_str,                  // 3: serial - filled with "picoflex" + board id
};

static uint16_t desc_str[33];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) return NULL;

        if (index == 3 && serial_str[0] == '\0') {
            pico_unique_board_id_t id;
            pico_get_unique_board_id(&id);
            strcpy(serial_str, PIFLEX_SERIAL_PREFIX);
            char *p = serial_str + strlen(serial_str);
            for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
                sprintf(p, "%02X", id.id[i]);
                p += 2;
            }
        }

        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) desc_str[1 + i] = str[i];
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
