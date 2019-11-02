/**
 * Copyright 2018 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

// Note, ATmega328p was used for development purposes and is not guaranteed to work.
// Use ATtiny85 for proper operation.
// ATtiny85 fuses -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
// ATmega328p fuses -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xfd:m

// ***** Configurable options ***** 
//#define SLEEP_TIMER_DEFAULT_DURATION (60L*60*6)
#define SLEEP_TIMER_DEFAULT_DURATION (30L)
//#define WAKEUP_TIMER_DURATION (60L*60*24)
#define WAKEUP_TIMER_DURATION (60L)

//#define FAST_FADE_TIME_MS 2000
#define FAST_FADE_TIME_MS 100
#define OFF_MIN_BRIGHT 0
#define ON_MAX_BRIGHT 0xff
#define DIMMER_STEP 10
#define DIMMER_MIN 200
// ********************************

// Not recommended to change config values beyond this point.
#define BUTTON_DEBOUNCE_MIN_SAMPLES 2
#define BUTTON_DEBOUNCE_MAX_SAMPLES 100
#define BUTTON_DEBOUNCE_HOLD_INITIATE_SAMPLES 200
#define BUTTON_DEBOUNCE_HOLD_CONTINUE_SAMPLES 50
#define PRESCALE 128

#define OVF_PER_UNIT ((F_CPU / PRESCALE) / 0x100) // 1 second

#if SLEEP_TIMER_DEFAULT_DURATION > WAKEUP_TIMER_DURATION
#error Sleep timer duration must be smaller than wakeup timer duration
#endif

#define FAST_FADE_STEP_DURATION (FAST_FADE_TIME_MS / ON_MAX_BRIGHT)

// ATmega328p
// PD3 - mode switch PCINT2
// PB1 - LED
// PD2 - button

// ATtiny85
// PB2 - mode switch PCINT2
// PB1 - LED
// PB0 - button

typedef enum {
    STATE_OFF,
    STATE_ON,
} state_t;

typedef enum {
    MODE_NORMAL,
    MODE_AUTO,
} mode_t;

#if _AVR_IOM328P_H_
// ATmega328p
#   define MODE_PIN    PIND
#   define BUTTON_PIN  PIND

#   define MODE_DDR    DDRD
#   define BUTTON_DDR  DDRD
#   define LED_DDR     DDRD

#   define MODE_PORT   PORTD
#   define BUTTON_PORT PORTD
#   define LED_PORT    PORTD

#   define MODE        PORTD3
#   define BUTTON      PORTD2
#   define LED         PORTD5
#else
// ATtiny85
#   define MODE_PIN    PINB
#   define BUTTON_PIN  PINB

#   define MODE_DDR    DDRB
#   define BUTTON_DDR  DDRB
#   define LED_DDR     DDRB

#   define MODE_PORT   PORTB
#   define BUTTON_PORT PORTB
#   define LED_PORT    PORTB

#   define MODE        PORTB2
#   define BUTTON      PORTB0
#   define LED         PORTB1
#endif

#define TIMER_ENABLED   (1<<0)
#define TIMER_TRIGGERED (1<<1)

static const uint8_t PROGMEM gamma8[256] = {
      0,   0,  0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   2,   2,   2,   2,
      3,   3,  3,   3,   4,   4,   4,   5,   5,   5,   6,   6,   6,   7,   7,   8,
      8,   8,  9,   9,  10,  10,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,
     16,  16, 17,  17,  18,  19,  19,  20,  20,  21,  22,  22,  23,  24,  24,  25,
     25,  26, 27,  27,  28,  29,  30,  30,  31,  32,  32,  33,  34,  35,  35,  36,
     37,  38, 38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  47,  48,  49,
     50,  51, 52,  53,  54,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,
     65,  66, 67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,
     81,  82, 83,  84,  85,  86,  87,  88,  89,  91,  92,  93,  94,  95,  96,  97,
     98, 100,101, 102, 103, 104, 105, 106, 108, 109, 110, 111, 112, 114, 115, 116,
    117, 118,120, 121, 122, 123, 125, 126, 127, 128, 130, 131, 132, 134, 135, 136,
    137, 139,140, 141, 143, 144, 145, 147, 148, 149, 151, 152, 153, 155, 156, 158,
    159, 160,162, 163, 164, 166, 167, 169, 170, 172, 173, 174, 176, 177, 179, 180,
    182, 183,185, 186, 187, 189, 190, 192, 193, 195, 196, 198, 199, 201, 202, 204,
    205, 207,209, 210, 212, 213, 215, 216, 218, 219, 221, 223, 224, 226, 227, 229,
    230, 232,234, 235, 237, 239, 240, 242, 243, 245, 247, 248, 250, 252, 253, 255,
};

typedef struct {
    uint8_t flags;
    uint32_t ovf_count;
    uint32_t ovf_max;
} timer_t;

static volatile state_t state;
static volatile mode_t mode;
static volatile timer_t wakeup_timer;
static volatile timer_t sleep_timer;

static uint8_t brightness = 0;
static uint8_t dimmer_max = ON_MAX_BRIGHT;
static int dimmer_dir = 1;

static void error(void) {

}

static mode_t get_mode(void) {
    if (MODE_PIN & _BV(MODE)) {
        return MODE_AUTO;
    } else {
        return MODE_NORMAL;
    }
}

__attribute__((always_inline))
static inline uint8_t _gamma_correct(const uint8_t input) {
    return pgm_read_byte(&gamma8[input]);
}

static void lights_off(void) {
    if (brightness == OFF_MIN_BRIGHT) {
        return;
    }
    for (int32_t i = brightness; i >= OFF_MIN_BRIGHT && state == STATE_OFF; i--) {
        OCR0B = brightness = _gamma_correct(i);
        _delay_ms(FAST_FADE_STEP_DURATION);
    }
    if (brightness == OFF_MIN_BRIGHT) {
        TCCR0A &= ~_BV(COM0B1);
    }
    brightness = OFF_MIN_BRIGHT;
}

static void lights_on(void) {
    if (brightness == dimmer_max) {
        return;
    }
    for (int32_t i = brightness; i <= dimmer_max && state == STATE_ON; i++) {
        OCR0B = brightness = _gamma_correct(i);
        TCCR0A |= _BV(COM0B1);
        _delay_ms(FAST_FADE_STEP_DURATION);
    }
}

static void lights_dim(uint8_t val) {
    if (brightness == val) {
        return;
    }
    if (brightness < val) {
        // fade up
        for (int32_t i = brightness; i <= val && state == STATE_ON; i++) {
            OCR0B = brightness = _gamma_correct(i);
            TCCR0A |= _BV(COM0B1);
            _delay_ms(FAST_FADE_STEP_DURATION);
        }
    } else if (brightness > val) {
        // fade down
        for (int32_t i = brightness; i >= val && state == STATE_ON; i--) {
            OCR0B = brightness = _gamma_correct(i);
            TCCR0A |= _BV(COM0B1);
            _delay_ms(FAST_FADE_STEP_DURATION);
        }
    }
    dimmer_max = val;
}

static void timer_tick(volatile timer_t *timer) {
    if (timer->flags & TIMER_ENABLED) {
        timer->ovf_count += 1;
        if (timer->ovf_count >= timer->ovf_max) {
            timer->flags &= ~TIMER_ENABLED;
            timer->flags |= TIMER_TRIGGERED;
        }
    }
}

static void timer_start(volatile timer_t *timer) {
    timer->flags &= ~TIMER_TRIGGERED;
    timer->ovf_count = 0;
    timer->flags |= TIMER_ENABLED;
}

static void timer_cancel(volatile timer_t *timer) {
    timer->flags &= ~(TIMER_TRIGGERED | TIMER_ENABLED);
}

static uint8_t timer_triggered(volatile timer_t *timer) {
    if (timer->flags & TIMER_TRIGGERED) {
        return 1;
    } else {
        return 0;
    }
}

static uint32_t timer_get_elapsed_time(volatile timer_t *timer) {
    return timer->ovf_count;
}

static void timer_set_duration(volatile timer_t *timer, uint32_t duration) {
    timer->ovf_max = OVF_PER_UNIT * duration;
}

static void _mode_interrupt(void) {
    mode_t new_mode = get_mode();
    if (new_mode != mode) {
        mode = new_mode;
    }
}

#if _AVR_IOM328P_H_ // ATmega328p
ISR(INT1_vect, ISR_BLOCK) {
    _mode_interrupt();
}
#else // ATtiny85
ISR(PCINT0_vect, ISR_BLOCK) {
    _mode_interrupt();
}
#endif

static volatile uint32_t ovf0_count = 0;
ISR(TIMER0_OVF_vect, ISR_BLOCK) {
    ovf0_count++;
}

ISR(TIMER1_OVF_vect, ISR_BLOCK) {
    timer_tick(&wakeup_timer);
    timer_tick(&sleep_timer);
}

static void button_check(uint8_t *pressed, uint8_t *held) {
    static uint8_t button_down = 0;
    static uint32_t debounce_start = 0;
    static uint32_t next_hold = 0;
    *pressed = 0;
    *held = 0;
    
    if (!(BUTTON_PIN & _BV(BUTTON))) {
        if (button_down == 0) {
            debounce_start = ovf0_count;
            next_hold = BUTTON_DEBOUNCE_HOLD_INITIATE_SAMPLES;
        }
        uint32_t ticks = ovf0_count - debounce_start;
        if (ticks >= next_hold) {
            while (next_hold <= ticks) {
                next_hold += BUTTON_DEBOUNCE_HOLD_CONTINUE_SAMPLES;
            }
            *held = 1;
        }
        button_down = 1;
    } else {
        if (button_down) {
            uint32_t ticks = ovf0_count - debounce_start;
            if (ticks >= BUTTON_DEBOUNCE_MIN_SAMPLES && ticks <= BUTTON_DEBOUNCE_MAX_SAMPLES) {
                *pressed = 1;
            }
        }
        button_down = 0;
    }
}

int main(void) {
    OSCCAL = 149;

    LED_DDR |= _BV(LED); // LED output
    MODE_DDR &= ~_BV(MODE); // mode switch input
    BUTTON_DDR &= ~_BV(BUTTON); // button input
    MODE_PORT |= _BV(MODE); // pull ups
    BUTTON_PORT |= _BV(BUTTON); // pull ups

    // external interrupt on falling edge
#if _AVR_IOM328P_H_
    // ATmega328p
    EICRA = _BV(ISC10) | _BV(ISC01);
    EIMSK = _BV(INT1);
    EIFR |= _BV(INTF0) | _BV(INTF1);
#else
    // ATtiny85
    MCUCR |= _BV(ISC01);
    GIMSK |= _BV(PCIE);
    GIFR |= _BV(PCIF);
    PCMSK |= _BV(PCINT2);
#endif

    mode = get_mode();
    state = STATE_OFF;

    wakeup_timer.flags = 0;
    wakeup_timer.ovf_count = 0;
    timer_set_duration(&wakeup_timer, WAKEUP_TIMER_DURATION);
    
    sleep_timer.flags = 0;
    sleep_timer.ovf_count = 0;
    timer_set_duration(&sleep_timer, SLEEP_TIMER_DEFAULT_DURATION);

    sei();

    // timer config
    OCR0B = 0;
    brightness = 0;

#if _AVR_IOM328P_H_
    // ATmega328p
    TIMSK1 = _BV(TOIE1);
    TIMSK0 = _BV(TOIE0);

    ICR1 = 4080;

    TCCR1A = _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12);
#error Figure this out...128 is not an option for this chip
#   if PRESCALE == 128
    TCCR1B |= _BV(CS12) | _BV(CS10);
#   else
#   error bad prescale value
#   endif

    TCCR0A = _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS00);

#else
    // ATtiny85
    GTCCR = 0;
    TIMSK = _BV(TOIE0) | _BV(TOIE1);

#   if PRESCALE == 128
    TCCR1 = _BV(CS13);
#   else
#   error bad prescale value
#   endif

    TCCR0A = _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS00);
#endif
   
    state_t old_state = state;
    mode_t old_mode = mode;
    while (1) {

        int16_t new_dimmer_max = dimmer_max;
        uint8_t pressed, held;
        button_check(&pressed, &held);
        if (pressed) {
            if (state == STATE_OFF) {
                state = STATE_ON;
            } else if (state == STATE_ON) {
                state = STATE_OFF;
            }
        } else if (held) {
            if (state == STATE_ON) {
                new_dimmer_max += DIMMER_STEP * dimmer_dir;
                if (new_dimmer_max > ON_MAX_BRIGHT) {
                    new_dimmer_max = ON_MAX_BRIGHT;
                    dimmer_dir *= -1;
                } else if (new_dimmer_max < DIMMER_MIN) {
                    new_dimmer_max = DIMMER_MIN;
                    dimmer_dir *= -1;
                }
            }
        }

        // timer triggers
        if (timer_triggered(&wakeup_timer)) {
            timer_cancel(&wakeup_timer);
            state = STATE_ON;
        }
        if (timer_triggered(&sleep_timer)) {
            timer_cancel(&sleep_timer);
            state = STATE_OFF;
        }
        
        state_t now_state = state;
        mode_t now_mode = mode;

        if (old_state == STATE_ON && old_state != state) {
            // STATE_ON - on exit
            timer_cancel(&sleep_timer);
            uint32_t dur = timer_get_elapsed_time(&sleep_timer);
            if (dur > 1800) {
                timer_set_duration(&sleep_timer, dur);
            }
        }

        // state transitions
        if (state == STATE_ON) {
            if (old_state != STATE_ON) {
                // STATE_ON - on entry
                if (now_mode == MODE_AUTO) {
                    // start timers
                    timer_start(&wakeup_timer);
                    timer_start(&sleep_timer);
                }
            }
            lights_on();
        } else if (state == STATE_OFF) {
            lights_off();
        }

        // dimmer transitions
        if (new_dimmer_max != dimmer_max) {
            lights_dim(new_dimmer_max);
        }

        // mode transitions
        if (old_mode != now_mode) {
            if (now_mode == MODE_NORMAL) {
                // MODE_NORMAL - on entry
                timer_cancel(&wakeup_timer);
                timer_cancel(&sleep_timer);
                timer_set_duration(&sleep_timer, SLEEP_TIMER_DEFAULT_DURATION);
            } else if (now_mode == MODE_AUTO) {
                // MODE_AUTO - on entry
                if (state == STATE_ON) {
                    timer_start(&sleep_timer);
                }
            }
        }

        if (old_state != state) {
        }

        old_state = now_state;
        old_mode = now_mode;
    }

    return 0;
}








