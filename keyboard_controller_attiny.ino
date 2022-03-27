// vim: syntax=c ts=4 sts=4 sw=4 expandtab

#include "Arduino.h"
#include "scancode_set_3.h"

/*

          ATTiny85 Running
          ,------_------,
          |             |
    Reset |1 RST   VCC 8| VCC
  PS2 CLK |2 D3     D2 7| Shift CLK
 PS2 DATA |3 D4     D1 6| Shift DATA
      GND |4 GND    D0 5| CPU Interrupt
          |             |
          `-------------'

       ATTiny85 Programming
          ,------_------,
          |             |
   Uno 10 |1 RST   VCC 8| VCC
          |2 D3     D2 7| Uno 13
          |3 D4     D1 6| Uno 12
      GND |4 GND    D0 5| Uno 11
          |             |
          `-------------'

*/

#define PORT            PORTB
#define DDR             DDRB
#define PIN             PINB
#define PS2_CLK_PIN     3
#define PS2_CLK_MASK    0b00001000
#define PS2_DATA_PIN    4
#define PS2_DATA_MASK   0b00010000
#define SHIFT_CLK_PIN   2
#define SHIFT_CLK_MASK  0b00000100
#define SHIFT_DATA_PIN  1
#define SHIFT_DATA_MASK 0b00000010
#define CPU_INT_PIN     0
#define CPU_INT_MASK    0b00000001

#define BAD_START_BIT   0xff
#define BAD_PARITY_BIT  0xfe
#define BAD_STOP_BIT    0xfd

#define PS2_BREAK       0xf0
#define PS2_EXTEND      0xe0
#define PS2_ACK         0xfa

#define PS2_CMD_RESET   0xff
#define PS2_CMD_SET_LED 0xed
#define PS2_CMD_SET_SCANCODE_SET 0xf0
#define PS2_CMD_SET_KEY_MAKEBREAK 0xfc
#define PS2_CMD_SET_ALL_TYPEMATIC_MAKE_BREAK 0xfa
#define PS2_CMD_ENABLE  0xf4

uint8_t flags = 0x00;
#define FLAGS_BREAK     0x01
#define FLAGS_CTRL      0x02
#define FLAGS_ALT       0x04
#define FLAGS_FUNC      0x08
#define FLAGS_SHIFT     0x10
#define FLAGS_NUMLOCK   0x20
#define FLAGS_CAPSLOCK  0x40
#define FLAGS_SCROLLLOCK 0x80

bool calc_parity(uint8_t n) {
   uint8_t y;
   y = n ^ (n >> 1);
   y = y ^ (y >> 2);
   y = y ^ (y >> 4);
   y = y ^ (y >> 8);
   //checking the rightmost bit
   if (y & 1)
      return 1;
   return 0;
}

bool send_kb_byte(uint8_t data_byte) {
    uint8_t mask = 0x01;

    // start with clock and data high
    PORT |= (PS2_CLK_MASK | PS2_DATA_MASK);
    // wait before starting to avoid too short of a high clock pulse
    delayMicroseconds(150);
    // switch to output mode for clock and data
    DDR |= (PS2_CLK_MASK | PS2_DATA_MASK);

    // bring clock line low for at least 100us
    PORT &= ~PS2_CLK_MASK;
    delayMicroseconds(150);

    // bring data line low
    PORT &= ~PS2_DATA_MASK;
    delayMicroseconds(10);

    // release clock line and switch it to input again
    PORT |= PS2_CLK_MASK;
    DDR &= ~PS2_CLK_MASK;

    // wait for kb to drop clock line
    while((PIN & PS2_CLK_MASK) > 0);

    // loop: set/reset data line to correct bit
    for(uint8_t i=0; i<=8; i++) {
        if(i < 8) {
            // regular bit
            if((data_byte & mask) > 0)
                PORT |= PS2_DATA_MASK;
            else
                PORT &= ~PS2_DATA_MASK;

            mask = (mask << 1);
        }
        else {
            // parity bit (odd parity)
            if(calc_parity(data_byte))
                PORT &= ~PS2_DATA_MASK;
            else
                PORT |= PS2_DATA_MASK;
        }

        // wait for clock high
        while((PIN & PS2_CLK_MASK) == 0);
        // wait for clock low
        while((PIN & PS2_CLK_MASK) > 0);
    }

    // release data line
    PORT |= PS2_DATA_MASK;
    // return data line to input mode
    DDR &= ~PS2_DATA_MASK;

    // wait for kb to bring data low
    while((PIN & PS2_DATA_MASK) > 0);

    // wait for kb to bring clock low
    while((PIN & PS2_CLK_MASK) > 0);

    // wait for data and clock to be released
    while((PIN & PS2_CLK_MASK) == 0);
    while((PIN & PS2_DATA_MASK) == 0);
}

uint8_t read_kb_byte() {
    uint8_t kb_byte = 0x00;
    bool this_bit;
    bool parity_bit;

    // wait for the keyboard to send the start bit
    // wait for PS2_CLK to go low
    while((PIN & PS2_CLK_MASK) > 0) {
        this_bit = (PIN & PS2_DATA_MASK) ? 1 : 0;
    }
    // clock is now low
    // check the start bit, should be zero
    if(this_bit) {
        return BAD_START_BIT;
    }
    // now loop for 8 bits of data
    kb_byte = 0x00;
    for(uint8_t i = 0; i < 8; i++) {
        // wait for clock to go high
        while((PIN & PS2_CLK_MASK) == 0);
        // read bit until clock goes low
        while((PIN & PS2_CLK_MASK) > 0) {
            this_bit = (PIN & PS2_DATA_MASK) ? 1 : 0;
        }
        // clock is now low
        // set this bit in kb_byte
        kb_byte = (kb_byte >> 1);
        if(this_bit) {
            kb_byte |= 0x80;
        }
    }

    // Read parity bit
    // wait for clock to go high
    while((PIN & PS2_CLK_MASK) == 0);
    // read bit until clock goes low
    while((PIN & PS2_CLK_MASK) > 0) {
        parity_bit = (PIN & PS2_DATA_MASK) ? 1 : 0;
    }
    // clock is now low, and parity bit is in parity_bit

    // Read stop bit
    // wait for clock to go high
    while((PIN & PS2_CLK_MASK) == 0);
    // read bit until clock goes low
    while((PIN & PS2_CLK_MASK) > 0) {
        this_bit = (PIN & PS2_DATA_MASK) ? 1 : 0;
    }
    // clock is now low, and stop bit is in this_bit

    // wait for clock to return high
    while((PIN & PS2_CLK_MASK) == 0);

    // check stop bit
    if(this_bit == 0) {
        return BAD_STOP_BIT;
    }

    // check parity
	if(calc_parity(kb_byte)) {
        // kb_byte has odd parity, so parity_bit should be zero
        if(parity_bit == 0) return kb_byte;
    }
    else {
        // kb_byte has even parity, so parity_bit should be one
        if(parity_bit == 1) return kb_byte;
    }
    return BAD_PARITY_BIT;
}

// Reset the keyboard (0xff) - will respond within 500-750ms with OK
// (0xaa) or error (0xfc). After reset, the keyboard will be in scancode set
// 2, typematic delay 500ms, typematic rate 10.9 cps, all keys
// typematic+make+break
bool kb_reset() {
    uint8_t response;
    send_kb_byte(PS2_CMD_RESET);
    response = read_kb_byte();
    if(response != PS2_ACK)
        return false;
    delay(100);
    response = read_kb_byte();
    return response == 0xaa;
}

// Send LED state (scroll=1, num=2, caps=4).
void kb_set_led() {
    uint8_t kb_flags = 0;
    if((flags & FLAGS_SCROLLLOCK) > 0)
        kb_flags |= 0x01;
    if((flags & FLAGS_NUMLOCK) > 0)
        kb_flags |= 0x02;
    if((flags & FLAGS_CAPSLOCK) > 0)
        kb_flags |= 0x04;
    send_kb_byte(PS2_CMD_SET_LED);
    send_kb_byte(kb_flags);
}

// Send 0xf0 (set scancode set)
// wait for 0xfa (ack)
// Send 0x03 (scancode set 3)
// wait for 0xfa (ack)
bool kb_set_mode(uint8_t mode) {
    uint8_t response;
    send_kb_byte(PS2_CMD_SET_SCANCODE_SET);
    response = read_kb_byte();
    if(response != PS2_ACK) {
        delay(500);
        return false;
    }
    send_kb_byte(mode);
    response = read_kb_byte();
    if(response != PS2_ACK) {
        delay(500);
        return false;
    }
    return true;
}

// Send 0xfa (set all keys typematic/make/break)
// wait for 0xfa
bool kb_set_all_typematic_make_break() {
    uint8_t response;
    send_kb_byte(PS2_CMD_SET_ALL_TYPEMATIC_MAKE_BREAK);
    response = read_kb_byte();
    if(response == PS2_ACK) {
        send_kb_byte(PS2_CMD_ENABLE);
        return true;
    }
    return false;
}

// Send 0xfc (set key type make/break)
// wait for 0xfa (ack)
// send set 3 make code(s), each are followed by an ack from the kb
// send invalid make code (e.g. a command code)
// kb returns to scanning
bool kb_set_key_make_break(uint8_t *keys) {
    uint8_t *ptr = keys;
    uint8_t response;
    send_kb_byte(PS2_CMD_SET_KEY_MAKEBREAK);
    response = read_kb_byte();
    if(response != PS2_ACK) {
        delay(500);
        return false;
    }
    while(*ptr > 0) {
        send_kb_byte(*ptr);
        response = read_kb_byte();
        if(response != PS2_ACK) {
            delay(500);
            return false;
        }
        ptr++;
    }
    send_kb_byte(PS2_CMD_SET_KEY_MAKEBREAK);
    send_kb_byte(PS2_CMD_ENABLE);
    return true;
}

void setup() {
    uint8_t no_typematic[] = {
		SC3_numLock,
		SC3_capsLock,
		SC3_leftShift,
		SC3_leftCtrl,
		SC3_leftGui,
		SC3_leftWindows,
		SC3_leftAlt,
		SC3_rightShift,
		SC3_rightCtrl,
		SC3_rightGui,
		SC3_rightWindows,
		SC3_rightAlt,
		0x00
	};

    pinMode(CPU_INT_PIN, INPUT);
    pinMode(PS2_CLK_PIN, INPUT_PULLUP);
    pinMode(PS2_DATA_PIN, INPUT_PULLUP);
    pinMode(SHIFT_CLK_PIN, OUTPUT);
    pinMode(SHIFT_DATA_PIN, OUTPUT);

    digitalWrite(SHIFT_CLK_PIN, 0);
    digitalWrite(SHIFT_DATA_PIN, 0);

    // Reset so we start in a known state
    while(!kb_reset());

    // Set keyboard in PS2 mode (mode 3). The PS2 scancode set is much simpler
    // than mode 2, as it doesn't use extended keycodes for as many keys.
    if(!kb_set_mode(0x03)) {
        while(1) {
            // on failure, blink numlock+capslock
            flags = (FLAGS_NUMLOCK | FLAGS_CAPSLOCK);
            kb_set_led();
            delay(250);
            flags = 0;
            kb_set_led();
            delay(250);
        }
    }

    // Start by enabling typematic + make + break on all keys
    if(!kb_set_all_typematic_make_break()) {
        while(1) {
            // on failure, blink nulock+scrollock
            flags = (FLAGS_NUMLOCK | FLAGS_SCROLLLOCK);
            kb_set_led();
            delay(250);
            flags = 0;
            kb_set_led();
            delay(250);
        }
    }

    // Disable typematic on modifier keys
    if(!kb_set_key_make_break(no_typematic)) {
        while(1) {
            // on failure, blink capslock+scrollock
            flags = (FLAGS_CAPSLOCK | FLAGS_SCROLLLOCK);
            kb_set_led();
            delay(250);
            flags = 0;
            kb_set_led();
            delay(250);
        }
    }

    // initialize flags
    flags = FLAGS_NUMLOCK;
    kb_set_led();
}

void loop() {
    uint8_t kb_byte = 0x00;
    uint8_t translated_byte = 0x00;
    bool is_extend = 0;

    // reset the break flag when starting
    flags &= ~FLAGS_BREAK;

    // grab a byte from the keyboard
    kb_byte = read_kb_byte();

    // Catch conditions where we need to wait for
    // a second byte from the keyboard, such as
    // breaks or extended scancodes
    switch(kb_byte) {
        case BAD_START_BIT:
        case BAD_STOP_BIT:
        case BAD_PARITY_BIT:
            // pause for 200ms so the keyboard flushes out
            // whatever data has yet to be sent.  Then restart
            // the polling loop.
            delay(200);
            return;
        case PS2_BREAK:
            flags |= FLAGS_BREAK;
            kb_byte = read_kb_byte();
            break;
        case PS2_EXTEND:
            is_extend = 1;
            kb_byte = read_kb_byte();
            break;
    }

    // Catch conditions that aren't actual keystrokes we
    // send to the CPU, such as shift/alt/ctrl, and update
    // flags accordingly
    switch(kb_byte) {
        case SC3_leftShift:
        case SC3_rightShift:
            if((flags & FLAGS_BREAK) == 0)  flags &= ~FLAGS_SHIFT;
            else                            flags |= FLAGS_SHIFT;
            return;
        case SC3_leftCtrl:
        case SC3_rightCtrl:
            if((flags & FLAGS_BREAK) == 0)  flags &= ~FLAGS_CTRL;
            else                            flags |= FLAGS_CTRL;
            return;
        case SC3_leftAlt:
        case SC3_rightAlt:
            if((flags & FLAGS_BREAK) == 0)  flags &= ~FLAGS_ALT;
            else                            flags |= FLAGS_ALT;
            return;
        case SC3_numLock:
            if((flags & FLAGS_BREAK) == 0) {
                flags ^= FLAGS_NUMLOCK;
                kb_set_led();
            }
            return;
        case SC3_capsLock:
            if((flags & FLAGS_BREAK) == 0) {
                flags ^= FLAGS_CAPSLOCK;
                kb_set_led();
            }
            return;
        case SC3_scrollLock:
            if((flags & FLAGS_BREAK) == 0) {
                flags ^= FLAGS_SCROLLLOCK;
                kb_set_led();
            }
            return;
		case SC3_f1:
		case SC3_f2:
		case SC3_f3:
		case SC3_f4:
		case SC3_f5:
		case SC3_f6:
		case SC3_f7:
		case SC3_f8:
		case SC3_f9:
		case SC3_f10:
		case SC3_f11:
		case SC3_f12:
            // With F-keys, the translator will end up returning normal
            // ASCII 1-0, ! and @, but with FLAGS_FUNC set.  After sending
            // the translated keycode + flags we clear this bit.
            flags |= FLAGS_FUNC;
            break;
    }
    // get translated key
	        
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLK_PIN, LSBFIRST, kb_byte);
    shiftOut(SHIFT_DATA_PIN, SHIFT_CLK_PIN, LSBFIRST, flags);

    // clear the FUNC flag before pulling the next keystroke
    flags &= ~FLAGS_FUNC;
}
