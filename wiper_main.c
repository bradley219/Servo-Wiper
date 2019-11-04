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
#include <avr/sleep.h>


// ATtiny85
#define SERVO_DDR  DDRB
#define SERVO_PORT PORTB
#define SERVO      PORTB3

#define SWITCH_PIN  PINB
#define SWITCH_DDR  DDRB
#define SWITCH_PORT PORTB
#define SWITCH      PORTB2

#define SERVO_POWER_DDR  DDRB
#define SERVO_POWER_PORT PORTB
#define SERVO_POWER      PORTB4

static void timer0_start(void) {
    // Pulse high
    SERVO_PORT |= _BV(SERVO);
    // Start timer
    TCCR0B = _BV(CS01) | _BV(CS00);
}

static void timer0_stop(void) {
    // Stop timer and reset counter
    TCCR0B &= ~(_BV(CS01) | _BV(CS00));
    TCNT0 = 0;
}

static volatile uint8_t sleep_requested = 0;
static volatile uint8_t sleep_request_count = 0;

static void cancel_sleep_request(void) {
    sleep_requested = 0;
    sleep_request_count = 0;
}

static void servo_power_on(void) {
    SERVO_POWER_PORT &= ~_BV(SERVO_POWER);
}

static void servo_power_off(void) {
    SERVO_POWER_PORT |= _BV(SERVO_POWER);
}

static void enable_pins(void) {
    SERVO_DDR |= _BV(SERVO);
    SWITCH_DDR &= ~_BV(SWITCH);
    SERVO_POWER_DDR |= _BV(SERVO_POWER);
}

static void disable_pins(void) {
    SERVO_DDR &= ~_BV(SERVO);
    SWITCH_DDR &= ~_BV(SWITCH);
    SERVO_POWER_DDR &= ~_BV(SERVO_POWER);
    SERVO_POWER_PORT &= ~_BV(SERVO_POWER);
}

ISR(TIMER0_COMPA_vect, ISR_BLOCK) {
    // Pulse low
    SERVO_PORT &= ~_BV(SERVO);
    timer0_stop();
    if (sleep_requested) {
        sleep_request_count++;
    }
}

ISR(TIMER1_OVF_vect, ISR_BLOCK) {
    timer0_start();
}

ISR(INT0_vect, ISR_BLOCK) {
    // wake up
    cancel_sleep_request();
    servo_power_on();
    enable_pins();
            
    // Disable INT0 interrupt
    GIMSK &= ~_BV(INT0);
}

#define SERVO_MIN 126
#define SERVO_MID 189
#define SERVO_MAX 253


static void _init(void) {

    // Init ports
    SERVO_PORT &= ~_BV(SERVO);
    SWITCH_PORT &= ~_BV(SWITCH);
    servo_power_on();
    enable_pins();
  
    // Timer 0 config
    // normal mode 
    // prescale 64 
    TCCR0A = 0;
    TCNT0 = 0;
    OCR0A = SERVO_MID;
    TIMSK |= _BV(OCIE0A);

    // Timer 1 config
    //TCCR0B = _BV(CS01) | _BV(CS00);
    // prescale 512
    TCCR1 = _BV(CS13) | _BV(CS11);
    TCNT1 = 0;
    OCR1C = 156; // 20ms period
    TIMSK |= _BV(TOIE1);
    
    sei();
}

static inline void _loop(void) {
    static int increment = 1;
    static int16_t value = SERVO_MIN;

    if (SWITCH_PIN & _BV(SWITCH)) {
        // Need to go to sleep after setting servo position
        sleep_requested = 1;
        OCR0A = SERVO_MID;
    } else { 
        cancel_sleep_request();

        value = OCR0A + increment;
        if (value > SERVO_MAX || value < SERVO_MIN) {
            increment *= -1;
            value = OCR0A + increment;
        }
        OCR0A = value;
    }
    if (sleep_requested) {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        cli();
        if (sleep_request_count >= 10) { // 10 * 10ms = 100ms for servo to move to home position
            servo_power_off();
            disable_pins();
            sleep_enable();
            sleep_bod_disable();
            
            // Enable INT0 interrupt
            GIMSK |= _BV(INT0);
    
            sei();
            sleep_cpu();
            sleep_disable();
        }
        sei();
    }
    _delay_ms(10);
}

int main(void) {
    _init();
    for (;;) {
        _loop();
    }
    return 0;
}








