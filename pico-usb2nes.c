/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv2 license, which unfortunately won't be
 * written for another century.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <hardware/i2c.h>
#include <pico/i2c_slave.h>

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "ws2812.pio.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include "pio_usb.h"
#include "tusb.h"

#include "neskbdinter.h"
#include "usb2famikb.h"

// unnecessary now?
//#include "kblayout.c"

// $4016 "out" from AV Famicom/NES, only 1 pin
#define NES_OUT 5
// $4017 OE $4017 strobe read
#define NES_JOY2OE 13
// $4016 OE $4016 strobe read - unnecessary
#define NES_JOY1OE 3
// $4017 "data" lines, five consecutive pins
#define NES_DATA 8

// I2C communication pins
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

// direct input or i2chost select
#define i2cHOST_ENABLE 4

// backup led
#define BACKUP_LED 14


#define MAX_BUFFER 16
#define WORD_SIZE 4

// extra config for devices in direct input mode
#define MSERELATIVE 1
#define HORILHAND 1
#define HORILOWSPD 0
#define SENDREPEATS 1


// configuration for PIO USB
// DPDM configuration D+ = pin 6, D- = pin 7
// configure used SMs around currently used SMs
// use pio1, it has available SMs
// dp pin, usb tx pio, usb tx sm, usb tx dma
// usb rx pio, usb rx sm, usb eop sm
#define PIO_USB_CONFIG                                                          \
    {                                                                           \
        6, 1, 0, 0,                                                            \
            1, 1, 2,                                                            \
            NULL, PIO_USB_DEBUG_PIN_NONE,                                       \
            PIO_USB_DEBUG_PIN_NONE, false, PIO_USB_PINOUT_DPDM                  \
    }

static const uint8_t modkeys[] = { 
    KEY_LEFTCTRL, KEY_LEFTSHIFT, KEY_LEFTALT, KEY_LEFTMETA,
    KEY_RIGHTCTRL, KEY_RIGHTSHIFT, KEY_RIGHTALT, KEY_RIGHTMETA
};

// Just use a 6 byte memory array for storing the latest data for the mouse
static struct
{
    uint8_t mem[6];
    uint8_t mem_address;
    bool mem_address_written;
    bool garbage_message;
} hostmsg;

static bool new_input_msg;
// FIFO buffer for keypresses for standard mode
// buffer length will be relatively small because under standard operation
// the NES is likely to be reading from the buffer very frequently
static uint8_t bufferindex = 0;
static uint8_t keybuffer[MAX_BUFFER];
static uint8_t kbbbindex = 0;
static uint8_t kbbackbuffer[MAX_BUFFER];
// transport buffer index
static uint8_t transbbindex = 0;
// mouse updates won't be buffered like the keyboard, if multiple updates come
// inbetween a frame, we want to put them together instead of stack them up
// oversizing the buffer type to mitigate overflow
static int16_t msebuffer[4];
// we want to store 
static int16_t mseinstbuf[4];
static uint8_t joypadinst = 0;

static bool NESinlatch = false;

static uint8_t usb2kbmode;
static bool i2chostmode = false;

static bool instrobe = false;

static uint32_t kbword = 0;
static uint32_t mseword = 0;
static uint32_t nesjoypad = 0;

// gonna use arrow keys for dpad
// x for A, z for B, w for start, q for select
static const uint8_t neskeys[] = { 0x1B, 0x1D, 0x14, 0x1A, 0x52, 0x51, 0x50, 0x4F };


// https://github.com/raspberrypi/pico-examples/blob/master/blink/blink.c
// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#elif defined(PICO_DEFAULT_WS2812_PIN)
    // for RP2040-Zero devices
    PIO pio = pio0;
    int sm = 1;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, 16, 1120000, true);
    pio_sm_put_blocking(pio0, 1, 0); // ensure LED off
    sleep_ms(3);
    return PICO_OK;
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#elif defined(PICO_DEFAULT_WS2812_PIN)
    // for RP2040-zero devices
    if (led_on) {
        pio_sm_put_blocking(pio0, 1, 0x22222222); // dullwhite
    } else {
        pio_sm_put_blocking(pio0, 1, 0); // off
    }
#endif
}
// -----------------------------------------------------------

// handle key input into the buffer or matrices
static void keycode_handler(uint8_t ascii) {
    // keyboard mouse host mode
    if ((bufferindex+1) == MAX_BUFFER) {
        for (int i = 1; i < MAX_BUFFER; i++) {
            keybuffer[i-1] = keybuffer[i];
        }
        keybuffer[bufferindex] = ascii;
    }
    else {
        keybuffer[bufferindex] = ascii;
        bufferindex++;
    }
    
}

// I2C configuration
static const uint I2C_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz
// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!hostmsg.mem_address_written) {
            // writes always start with the memory address
            // the first value here is a len, ignore
            hostmsg.mem_address = i2c_read_byte_raw(i2c);
            // host should always address addr 0 in the buffer
            if (hostmsg.mem_address != 0){
                hostmsg.garbage_message = true;
            } else {
                hostmsg.garbage_message = false;
            }
            hostmsg.mem_address_written = true;
        } else {
            // if it is garbage we just read the values
            // but don't put them in memory
            if (hostmsg.garbage_message) {
                i2c_read_byte_raw(i2c);
            }
            else { // put thew values into buffer
                hostmsg.mem[hostmsg.mem_address] = i2c_read_byte_raw(i2c);
                hostmsg.mem_address = (hostmsg.mem_address + 1) % 6;
            }
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte_raw(i2c, hostmsg.mem[hostmsg.mem_address]);
        hostmsg.mem_address = (hostmsg.mem_address + 1) % 6;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        if (!hostmsg.garbage_message) {
            // parse the value from mem[1] if not 0x00
            if (hostmsg.mem[1] != 0x00) {
                keycode_handler(hostmsg.mem[1]);
            }
            // only update mouse buffer if mouse is "present"
            if ((hostmsg.mem[2] & 32) == 32) {
                // and the first byte with the current value
                // this means if a button is pressed, it will stay "pressed"
                // until the NES polls it (probably next frame)
                msebuffer[0] |= hostmsg.mem[2];
                // if true, we are in relative mode
                if ((msebuffer[0] & 8) == 8 && new_input_msg) {
                    msebuffer[1] += (int8_t)hostmsg.mem[3];
                    msebuffer[2] += (int8_t)hostmsg.mem[4];
                } else {
                    msebuffer[1] = (int8_t)hostmsg.mem[3];
                    msebuffer[2] = (int8_t)hostmsg.mem[4];   
                }
                // some wheel movements or middle button events could
                // be missed. target for improvement later
                msebuffer[3] |= hostmsg.mem[5];
            }
            new_input_msg = true;
        }
        
        hostmsg.mem_address_written = false;
        
        break;
    default:
        break;
    }
}

// when mouse data is sent to the NES, update relevant buffer data
static void update_mouse_data() {
    // if there is new data from the host
    if (new_input_msg) {
        if (i2chostmode) {
            // actually update button values
            msebuffer[0] = hostmsg.mem[2];
            msebuffer[3] = hostmsg.mem[5];
        } else {
            msebuffer[0] = mseinstbuf[0];
            msebuffer[1] = mseinstbuf[1];
            msebuffer[2] = mseinstbuf[2];
            msebuffer[3] = mseinstbuf[3] & 0x87;
        }
        new_input_msg = false;
    }
}

// IRQ handler for OE lines to shift data
void pio_IRQ_handler() {

    if (pio_interrupt_get(pio0, 3)) {
        mseword = mseword << 1;
        kbword = kbword << 1;
        nesjoypad = nesjoypad >> 1;

        pio_interrupt_clear(pio0, 3);
    }
}


void nes_handler_thread() {
    pio_set_irq0_source_enabled(pio0, pis_interrupt3, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_IRQ_handler);
    
    irq_set_enabled(PIO0_IRQ_0, true);

    usb2famikb_init(NES_OUT, NES_JOY1OE, NES_JOY2OE, NES_DATA, usb2kbmode);

    for (;;) {

        uint8_t nesread = (pio0->intr >> 8) & 0x0F;
        
        // read the strobe value, if was previously in strobe
        // exit strobe if no longer in strobe
        uint8_t strobe = nesread & 1;
        if (!strobe && instrobe) {
            instrobe = false;
        }

        // check for strobe signal and latch the buffers
        if (strobe && !instrobe) {  //  reset keyboard row/strobe mouse
            instrobe = true;
            // check if latched for buffer context
            NESinlatch = true;

            kbword = 0x00000000;
            mseword = 0x00000000;
            nesjoypad = joypadinst;
            // load the four oldest buffered values
            for (int i = 0; i < WORD_SIZE; i++) {
                kbword = kbword << 8;
                kbword += keybuffer[i];
                // mouse doesn't actually have a history
                // just get the latest values
                mseword = mseword << 8;
                mseword += (uint8_t) msebuffer[i];
            }
            int c = WORD_SIZE;
            if (bufferindex < WORD_SIZE) {
                c = bufferindex;
            }
            for (int i = c; i < MAX_BUFFER; i++) {
                keybuffer[i-c] = keybuffer[i];
            }
            bufferindex = bufferindex - c;

            update_mouse_data();

            NESinlatch = false;
        }

        uint32_t serialout = 2; // effectively D1 on the pico, doesn't go to nes
        // push next mouse bit in
        serialout += (~mseword & 0x80000000) >> 27;
        // push the next keyboard bit in
        serialout += (~kbword & 0x80000000) >> 28;
        // push the joypad bit, this could be connected to D0 or D1 on the port
        serialout += (~nesjoypad & 0x01);
    
        usb2famikb_putkb(serialout);
    }

}

int main() {
    //  one rp2040-zero would not reliably set clock speed at default
    //  voltage. slight (0.05V) over-volt set to increase reliablity
    //  core temperatures will not be an issue (maybe slightly warm
    //  to the touch)
    //vreg_set_voltage(12);
    //sleep_ms(15);
    // need a clock speed that is a multiple of 12,000
    //set_sys_clock_khz(264000, true);
    // even stock clock speed of 120000 seems work for at least most
    // keyboard applications. higher clock will help ensure stability
    set_sys_clock_khz(168000, true);

    //  0: serialized mode - only mode supported
    usb2kbmode = 0;

    gpio_init(i2cHOST_ENABLE);
    gpio_set_dir(i2cHOST_ENABLE, GPIO_IN);
    gpio_pull_down(i2cHOST_ENABLE);
    
    i2chostmode = gpio_get(i2cHOST_ENABLE);

    // prepare buffers
    for (int i = 0; i < MAX_BUFFER; i++) {
        keybuffer[i] = 0x00;
    }
    for (int i = 1; i < 4; i++) {
        msebuffer[i] = 0x00;
        mseinstbuf[i] = 0x00;
    }
    // set only the device id in the buffer so if the NES
    // strobes for update before data received it will know
    // the interface is present
    msebuffer[0] = mseinstbuf[0] = 0x06;
    joypadinst = 0x00;

    multicore_reset_core1();
    //  run the NES handler on seperate core
    multicore_launch_core1(nes_handler_thread);

    // turn on LED to show device has booted fine
    pico_led_init();
    pico_set_led(true);

    if (i2chostmode) {
        gpio_init(I2C_SDA_PIN);
        gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA_PIN);

        gpio_init(I2C_SCL_PIN);
        gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SCL_PIN);

        
        i2c_init(i2c0, I2C_BAUDRATE);
        // configure I2C0 for slave mode
        i2c_slave_init(i2c0, I2C_ADDRESS, &i2c_slave_handler);

        // loop forever now
        for (;;) {
            if ((transbbindex != kbbbindex) && !NESinlatch){
                // if the buffer is full, don't do anything yet
                if ((bufferindex+1) < MAX_BUFFER) {
                    keybuffer[bufferindex] = kbbackbuffer[transbbindex];
                    bufferindex++;

                    transbbindex = (transbbindex + 1) % MAX_BUFFER;
                } 
            }
            // gets a little ansy without a little sleep here
            sleep_ms(1);
        }
        
    }
    else {
        mseinstbuf[0] |= MSERELATIVE << 3;
        sleep_ms(10);

        // Use tuh_configure() to pass pio configuration to the host stack
        // Note: tuh_configure() must be called before
        pio_usb_configuration_t pio_cfg = PIO_USB_CONFIG;
        tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

        // To run USB SOF interrupt in core0, init host stack for pio_usb (roothub
        // port1) on core0
        tuh_init(1);

        while (true) {
            tuh_task(); // tinyusb host task
        }
    }

}

//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
    (void)desc_report;
    (void)desc_len;

    // Interface protocol (hid_interface_protocol_enum_t)
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    // Receive report from boot keyboard & mouse only
    // tuh_hid_report_received_cb() will be invoked when report is available
    if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD || itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
        switch (itf_protocol) {
            case (HID_ITF_PROTOCOL_KEYBOARD):
                mseinstbuf[0] |= 0x10;
                break;
            case (HID_ITF_PROTOCOL_MOUSE):
                mseinstbuf[0] |= 0x20;
                break;
        }
        new_input_msg = true;

        //  set up report receiving
        tuh_hid_receive_report(dev_addr, instance);

    }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
    switch (itf_protocol) {
        case (HID_ITF_PROTOCOL_KEYBOARD):
            mseinstbuf[0] &= 0xEF;
            break;
        case (HID_ITF_PROTOCOL_MOUSE):
            mseinstbuf[0] &= 0xDF;
            break;
    }
    new_input_msg = true;
}

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
    for(uint8_t i=0; i<6; i++) {
        if (report->keycode[i] == keycode)  return true;
    }

    return false;
}

// check if a keycode is missing from prev report
static inline void find_releases_in_report(hid_keyboard_report_t const *prev_report, hid_keyboard_report_t const *report)
{
    for(uint8_t i=0; i<6; i++)
    {
        if (prev_report->keycode[i] == 0x00) { break; }
        if (prev_report->keycode[i] != report->keycode[i]) {
            keycode_handler(prev_report->keycode[i] + 0x80);
            for (uint8_t j = 0; j < 8; ++j) {
                if (prev_report->keycode[i] == neskeys[j]) {
                    if (joypadinst & (1 << j)) {
                        joypadinst ^= 1 << j;
                    }
                    break;
                }
            }
            break;
        }
    }
}

// check if modifier keys have changed
static inline void update_modifiers(hid_keyboard_report_t const *prev_report, hid_keyboard_report_t const *report)
{
    uint8_t c = 1;
    for(uint8_t i=0; i<8; i++)
    {
        if ((prev_report->modifier & c) != (report->modifier & c)){
            if ((report->modifier & c) == 0){ // modifier released
                keycode_handler(modkeys[i] + 0x80);
            } else {
                keycode_handler(modkeys[i]);
            }
        }
        c <<= 1;
    }
}

// process the kbd report and send to keycode handler
static void process_kbd_report(hid_keyboard_report_t const *report)
{
    static hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released

    // keycode positions change when released, so need to check all
    find_releases_in_report(&prev_report, report);
    // check if modifier keys have changed
    update_modifiers(&prev_report, report);

    for(uint8_t i=0; i<6; i++)
    {
        uint8_t keycode = report->keycode[i];
        if ( keycode )
        {
            if (find_key_in_report(&prev_report, keycode)) {
                // ignore for now would like a repeat thing
            } else {
                keycode_handler(keycode);
                for (uint8_t j = 0; j < 8; ++j) {
                    if (keycode == neskeys[j]) {
                        joypadinst |= 1 << j;
                        break;
                    }
                }
            }
        }
    }

    prev_report = *report;
}

// process the mouse report and insert into buffers
static void process_mouse_report(hid_mouse_report_t const *report)
{
    uint8_t temp = mseinstbuf[0] & 0x3F;
    temp |= (report->buttons & MOUSE_BUTTON_LEFT) << 7;
    temp |= (report->buttons & MOUSE_BUTTON_RIGHT) << 5;
    mseinstbuf[0] = temp;
    msebuffer[0] |= mseinstbuf[0];

    temp = 0x00;
    temp |= (report->buttons & MOUSE_BUTTON_MIDDLE) << 5;
    temp |= (report->wheel & 0x0F) << 3;
    temp |= HORILHAND << 1;
    temp |= HORILOWSPD;
    mseinstbuf[3] = temp;
    msebuffer[3] |= mseinstbuf[3];

    if (MSERELATIVE) {
        if (new_input_msg) {
            msebuffer[1] += report->x;
            msebuffer[2] += report->y;
            mseinstbuf[1] = report->x;
            mseinstbuf[2] = report->y;
        } else {
            msebuffer[1] = mseinstbuf[1] = report->x;
            msebuffer[2] = mseinstbuf[2] = report->y;
        }
    } else {
        mseinstbuf[1] += report->x;
        if (mseinstbuf[1] < 0) { mseinstbuf[1] = 0; }
        if (mseinstbuf[1] > 255) { mseinstbuf[1] = 255; }

        mseinstbuf[2] += report->y;
        if (mseinstbuf[2] < 0) { mseinstbuf[2] = 0; }
        if (mseinstbuf[2] > 255) { mseinstbuf[2] = 255; }
        
        msebuffer[1] = mseinstbuf[1];
        msebuffer[2] = mseinstbuf[2];
    }

    new_input_msg = true;

}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
    (void) len;
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    switch(itf_protocol)
    {
        case HID_ITF_PROTOCOL_KEYBOARD:
        process_kbd_report((hid_keyboard_report_t const*) report );
        break;

        case HID_ITF_PROTOCOL_MOUSE:
        process_mouse_report((hid_mouse_report_t const*) report );
        break;

        default: break;
    }

    // continue to request to receive report
    tuh_hid_receive_report(dev_addr, instance);
}


#pragma GCC pop_options