// src/main.c
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "bsp/board.h"
#include "tusb.h"
#include "btstack.h"
#include "pico/btstack_run_loop_async_context.h"
#include "btstack_memory.h"
#include "hci.h"
#include "pico/btstack_cyw43.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "bth_device.h"

static btstack_packet_callback_registration_t hci_event_callback_registration;

void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);

int main() {
    board_init();

    if (cyw43_arch_init()) {
        // Initialization failed, blink LED rapidly
        while(true) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); sleep_ms(50);
        }
    }

    // Initialise BTstack
    if (!btstack_cyw43_init(cyw43_arch_async_context())) {
        printf("Failed to init BTstack CYW43\n");
        // Handle error, e.g., blink LED rapidly
        while(true) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); sleep_ms(50);
        }
    }

    // inform about BTstack state
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // turn on bluetooth!
    hci_power_control(HCI_POWER_ON);

    // Initialize the TinyUSB device stack
    tusb_init();

    // Main firmware loop
    while (1) {
        // Keep the CYW43 and TinyUSB tasks running
        async_context_poll(cyw43_arch_async_context());
        tud_task(); // TinyUSB task scheduler
    }
}

// UPSTREAM: CYW43 -> Pico -> Host PC
void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    // tud_bt_hci_ready() check removed as function does not exist

    switch (packet_type) {
        case 0x04: // HCI Event
            tud_bt_event_send(packet, size);
            break;
        case 0x02: // HCI ACL Data
            tud_bt_acl_data_send(packet, size);
            break;
    }
    return;
}

// DOWNSTREAM: Host PC -> Pico -> CYW43
// The original `tud_bt_hci_rx_cb` function is not compatible with TinyUSB's BTH device class API.
// We have implemented `tud_bt_hci_cmd_cb` and `tud_bt_acl_data_received_cb` instead.
// These functions will receive parsed HCI commands and ACL data from the USB host.
// They will then forward these to the BTstack HCI transport.

void tud_bt_hci_cmd_cb(void *hci_cmd, size_t cmd_len) {
    hci_transport_cyw43_instance()->send_packet(HCI_COMMAND_DATA_PACKET, (uint8_t*)hci_cmd, cmd_len);
}

void tud_bt_acl_data_received_cb(void *acl_data, uint16_t data_len) {
    hci_transport_cyw43_instance()->send_packet(HCI_ACL_DATA_PACKET, (uint8_t*)acl_data, data_len);
}

// --- USB Descriptors ---
#define USB_VID   0x2E8A // Raspberry Pi
#define USB_PID   0x000C // A new PID for this project
#define USB_BCD   0x0200

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_WIRELESS_CONTROLLER,
    .bDeviceSubClass    = 0x01,
    .bDeviceProtocol    = 0x01,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_BTH_DESC_LEN)
#define EPNUM_BT_EVT      0x81
#define EPNUM_BT_ACL_OUT  0x02
#define EPNUM_BT_ACL_IN   0x82

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_BTH_DESCRIPTOR(0, 0, EPNUM_BT_EVT, 16, 0x01, EPNUM_BT_ACL_OUT, EPNUM_BT_ACL_IN, 64, 0, 0)
};

char const* string_desc_arr[] = {
    (char[]) {0x09, 0x04}, "Raspberry Pi", "Pico W Bluetooth Dongle", "0000-PICO-HCI-0000",
};

uint8_t const* tud_descriptor_device_cb(void) { return (uint8_t const*) &desc_device; }
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) { (void) index; return desc_configuration; }

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    static uint16_t _desc_str[32];
    uint8_t chr_count;

    if (index == 0) { memcpy(&_desc_str[1], string_desc_arr[0], 2); chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char* str = string_desc_arr[index];
        chr_count = strlen(str); if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) { _desc_str[1 + i] = str[i]; }
    }
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}
