#include "Arduino.h"
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

/*
 * Capture PWM signal generated on d6 pin by timer1 through d8 pin.
 * Put 100n capacitor between d6 and d8 pins.
 * 
 * The AVR cannot reset the timer on capture event, so we let the timer continue count after capture interrunt
 * we handle timer overflow event and setup 17th bit in current_capture variable
 * When the capture interrupt fires again, we save ISR1 valut into curr_capture variable preserving 17th bit.
 * Then to get the pulse width we substract previous capture value
 */

const uint8_t CALIB_PIN     = 6;                // The reed switch that connect calibration capacitor
const uint8_t SHORT_PIN     = 7;                // The reed switch that shorts-up the internal LC cirquit
const uint8_t CAPTURE_PIN   = 8;                // Do not change, this is timer 1 capture pin
const uint8_t MODE_PIN      = 9;                // HIGH - Capacitance, LOW - inductance

// The LCD 0802 parallel interface
const uint8_t LCD_RS_PIN    = 4;
const uint8_t LCD_E_PIN     = 5;
const uint8_t LCD_DB4_PIN   = A0;
const uint8_t LCD_DB5_PIN   = A1;
const uint8_t LCD_DB6_PIN   = A2;
const uint8_t LCD_DB7_PIN   = A3;

// The buffer for pulse data
#define         BUFF_SZ     8
volatile static uint16_t    pulse[BUFF_SZ];
volatile static uint8_t     pulse_index = 0;    // Index of the pulse array
volatile static bool        pulse_ready = false;// The whole buffer has been filled
volatile static uint32_t    curr_capture  = 0;  // The current timer capture. 17-th bit can be setup by the overflow interrupt
volatile static uint16_t    prev_capture  = 0;  // Previuos value of timer captured value

// Oscillation periods for LC circuit and LC+Ccal. (with calibration capacitor) circuit, ticks
static double   ticks_LC    = 0.0;
static double   ticks_LCC   = 0.0;

const double    cal_cap_uF  = 0.001;                                    // The calibration capacitance (micro farads)
const double    four_pi_sq  = 39.4784176043574344753379639995;          // 4 * PI^2
const uint32_t  OPEN_TICKS  = 330000;                                   // The sum of ticks in the pulse[] when the circuit is open

LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);

void initCapture(void) {
    // Input Capture setup
    // ICNC1: Enable Input Capture Noise Canceler
    // ICES1: = 0 for trigger on falling edge
    // CS10:  = 1 set prescaler to 1x system clock (F_CPU)
    TCCR1A = 0;
    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (1<<CS10);
    TCCR1C = 0;
    // Interrupt setup
    // ICIE1: Input capture
    // TOIE1: Timer1 overflow
    TIFR1 = (1<<ICF1) | (1<<TOV1);              // clear pending interrupts
    TIMSK1 = (1<<ICIE1) | (1<<TOIE1);           // enable interupts

    // Set up the Input Capture pin, ICP1, Arduino Uno pin 8
    pinMode(CAPTURE_PIN, INPUT);
    digitalWrite(CAPTURE_PIN, 0);               // floating may have 60 Hz noise on it.
}

// Timer1 overflow interrupt handler
ISR(TIMER1_OVF_vect) {
    curr_capture = 0x10000;                     // setup 17-th bit
}

// Capture interrupt handler
ISR(TIMER1_CAPT_vect) {
    TIMSK1 &= ~(1<<ICIE1);                      // Disable this interrupt handler
    curr_capture &= 0x10000;                    // Only 17th bit is matters, clear other bits
    curr_capture |= ICR1;                       // Load timer1 capture value
    pulse[pulse_index] = curr_capture - prev_capture;
    if (++pulse_index >= 8) {
        pulse_index = 0;
        pulse_ready = true;
    }
    prev_capture = curr_capture & 0xFFFF;
    TIMSK1 |= (1<<ICIE1);                       // Enable this interrupt handler
}

// Absolute difference of two positive values
uint16_t abs_diff(uint16_t a, uint16_t b) {
    if (a >= b) {
        return a - b;
    } else {
        return b - a;
    }
}

// Oscillation period, ticks
uint32_t oscTicksSumm(bool reset) {
    if (reset) {
        pulse_ready = pulse_index = 0;          // Clear up the buffer
        while (!pulse_ready) ;                  // Wait till the whole buffer has been filled up
    }
    uint32_t summ = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        summ    = 0;
        uint16_t min_v  = 65535;
        uint16_t max_v  = 0;
        for (uint8_t i = 0; i < BUFF_SZ; ++i) {
            uint16_t p = pulse[i];
            if (p > max_v) max_v = p;
            if (p < min_v) min_v = p;
            summ += p;
        }
        if (abs_diff(min_v, max_v) < 100) break;
        lcd.clear();
        pulse_ready = pulse_index = 0;          // Clear up the buffer
        delay(1000);                            // Wait till the whole buffer has been filled up
    }
    return summ;
}

// The square of the period, microseconds^2. Argument - period in ticks
double periodSquare(double P) {
    P /= 16.0;                                  // Convert ticks to micro seconds
    P *= P;
    return P;
}

// Frequently used coefficient:  (P2^2 - P1^2) / P1^2
double pCoeff(double P1, double P2) {
    P1 *= P1;
    P2 *= P2;
    double r  = (P2 - P1) / P1;
    if (r < 0) r = 0;
    return r;
}

void calibrate(void) {
    while (true) {
        digitalWrite(SHORT_PIN, HIGH);              // Short the external socket
        delay(100);
        // Period of the internal oscillation circuit, ticks
        ticks_LC = (double)oscTicksSumm(true) / (double)BUFF_SZ;
        digitalWrite(CALIB_PIN, HIGH);              // Connect the calibration capacitor
        delay(100);
        // Period of the internal oscillation circuit plus calibration capacitor, ticks
        ticks_LCC = (double)oscTicksSumm(true) / (double)BUFF_SZ;
        digitalWrite(CALIB_PIN, LOW);               // Disconnect the calibration capacitor
        digitalWrite(SHORT_PIN, LOW);               // Open the external socket
        delay(100);
        if (ticks_LCC > ticks_LC) break;
        delay(1000);
    }
    while (!pulse_ready) ;                          // Wait till the whole buffer has been filled up
    double int_cap   = cal_cap_uF / pCoeff(ticks_LC, ticks_LCC);
}

void setup() {
    lcd.begin(8, 2);
    lcd.clear();
    pinMode(SHORT_PIN, OUTPUT);
    pinMode(CALIB_PIN, OUTPUT);
    digitalWrite(SHORT_PIN, LOW);
    digitalWrite(CALIB_PIN, LOW);
    pinMode(MODE_PIN, INPUT);
    initCapture();
    calibrate();
    lcd.clear();
}

void printValue(double value) {
    char sym = 'u';
    if (value > 1000.0) {
        value /= 1000;
        sym = 'm';
    }

    char buff[9];
    dtostrf(value, 6, 1, buff);
    uint8_t len = strlen(buff);
    buff[len++] = sym;
    buff[len++] = 'H';
    buff[len]   = '\0';
    lcd.print(F("Induct."));
    lcd.setCursor(0, 1);
    lcd.print(buff);
}

void checkInductance(void) {
    static uint16_t ticks_prev  = 0;            // previous Period of the oscillation, including external inductance or capacitance, ticks
    static uint32_t calibrate_ms = 0;
    
    uint16_t ticks_new    = oscTicksSumm(false);// Period of the internal oscillation circuit plus external thing, ticks
    if (ticks_new >= OPEN_TICKS) {
        lcd.clear();
        lcd.print(F(" NO"));
        lcd.setCursor(0, 1);
        lcd.print(F("value"));
        return;
    }

    double ticks     = (double)ticks_new / (double)BUFF_SZ;
    double pi_coeff  = 1 / (four_pi_sq * cal_cap_uF);                    // 1 / (4 * PI^2 * Cc)
    double int_ind   = (periodSquare(ticks_LCC) - periodSquare(ticks_LC)) * pi_coeff;
    double ext_ind   = int_ind * pCoeff(ticks_LC, ticks);

    printValue(ext_ind);
    if (millis() >= calibrate_ms || (ticks_prev && abs_diff(ticks, ticks_prev) > (BUFF_SZ >> 1))) {
        calibrate_ms = millis() + 10000;
        calibrate();
    }
    ticks_prev = ticks;
}

void loop() {
    checkInductance();
    delay(1000);
}
