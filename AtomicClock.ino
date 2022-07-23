#include "AtomicClock.h"

#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <SparkFunDS3234RTC.h>

#define CENTURY 2000
#define DST_ENABLED 1

// WWVB reference https://www.nist.gov/sites/default/files/documents/2017/04/28/SP-432-NIST-Time-and-Frequency-Services-2012-02-13.pdf
// Indices for parts of WWVB frame
enum
{
    FPRM, // Frame reference marker: .8L+.2H
    FPUU, // Unweighted: .2L+.8H
    // d1: .5L+.5H / 0 = .2L+.8H
    FPM1, // 10 minutes
    FPM2, //  1 minutes
    FPH1, // 10 hours
    FPH2, //  1 hours
    FPD1, // 100 days
    FPD2, // 10 days
    FPD3, //  1 days
    FPUS, // UTC sign
    FPUC, // UTC correction
    FPY1, // 10 years
    FPY2, //  1 years
    FPLY, // Leap year
    FPLS, // Leap second
    FPDS, // Daylight saving time
    FPEF
}; // End of frame same as FPRM
// Order of received frame, one per second
const byte FramePattern[] =
    // .0   .1   .2   .3   .4   .5   .6   .7   .8   .9
    /*0.*/ {FPRM, FPM1, FPM1, FPM1, FPUU, FPM2, FPM2, FPM2, FPM2, FPRM,
            /*1.*/ FPUU, FPUU, FPH1, FPH1, FPUU, FPH2, FPH2, FPH2, FPH2, FPRM,
            /*2.*/ FPUU, FPUU, FPD1, FPD1, FPUU, FPD2, FPD2, FPD2, FPD2, FPRM,
            /*3.*/ FPD3, FPD3, FPD3, FPD3, FPUU, FPUU, FPUS, FPUS, FPUS, FPRM,
            /*4.*/ FPUC, FPUC, FPUC, FPUC, FPUU, FPY1, FPY1, FPY1, FPY1, FPRM,
            /*5.*/ FPY2, FPY2, FPY2, FPY2, FPUU, FPLY, FPLS, FPDS, FPDS, FPEF};
#define FRAME_SIZE 60

// Receiver module http://canaduino.ca/downloads/60khz.pdf
// Receiver IC http://canaduino.ca/downloads/MAS6180C.pdf
/*
P.2 Note.2 OUT = VSS(low) when carrier amplitude at maximum;
OUT = VDD(high) when carrier amplitude is reduced (modulated)
P.7 Table.5 Recommended pulse width recognition limits for WWVB
Symbol  Min Max Unit
T 200ms 100 300 ms
T 500ms 400 600 ms
T 800ms 700 900 ms
*/
#define RADIO_POWERDOWN_PIN 6 // P1
#define RADIO_IN_PIN 7        // T
uint8_t radioPort, radioBit;

#define SAMPLE_HZ 50 // must be a factor of 62500: 2, 4, 5, 10, 20, 25, 50, 100, 125
#define CODE_N 0
#define CODE_U 1
#define CODE_W 2
#define CODE_P 3
#define CODE_X 4
byte code = CODE_N;
unsigned long cyclesSinceTimeSet = 0x80000000;

// Moving average filter http://www.dspguide.com/CH15.PDF
#define AVERAGING_LENGTH 10
uint8_t past[AVERAGING_LENGTH];
uint8_t avg, xpast, modcount, dur, pr;
#define EMPTY 0xFF

/* Timer 1 interrupt to measure signal
http://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
Timer0 8bit used for the timer functions, like delay(), millis() and micros()
Timer1 16bit the Servo library uses timer1 on Arduino Uno (timer5 on Arduino Mega)
Timer2 8bit the tone() function uses timer2
Timer 3,4,5 16bit only available on Arduino Mega boards
*/
ISR(TIMER1_COMPA_vect)
{
    // OUT = VSS(low) when carrier amplitude at maximum;
    // OUT = VDD(high) when carrier amplitude is reduced (modulated)
    //                  (-------digitalRead(RADIO_IN_PIN)--------)
    uint8_t modulated = (*portInputRegister(radioPort) & radioBit) ? 1 : 0;
    // Moving average filter (does not need to actually divide for an average)
    avg += modulated - past[xpast];
    past[xpast] = modulated;
    if (++xpast >= AVERAGING_LENGTH)
        xpast = 0;
    pr = avg;
    if (avg >= AVERAGING_LENGTH * 5 / 10)
    {
        ++modcount;
    }
    else
    {
        if (modcount)
        {
            dur = modcount;
            modcount = 0;
        }
    }
}

// Sparkfun DeadOn Real Ttime Clock https://learn.sparkfun.com/tutorials/deadon-rtc-breakout-hookup-guide
#define RTC_SELECT_PIN 10
#define RTC_INTERRUPT_PIN 2
// GND - GND
// VCC - 5V
// SQW - D2
// CLK - D13  ** conflicts with LED_BUILTIN!
// MISO - D12
// MOSI - D11
// SS - D10

int zoneHours = -6; // Dallas time

uint8_t frame[FRAME_SIZE], frameIndex = 0;
short decode[FPEF]; // accumulated parts of frame
bool timeSet = false;
byte daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
//                     JanFebMarAprMayJunJulAugSepOctNovDec
int wrap, n;

uint64_t now;
uint64_t lastDisplayWrite;

Adafruit_7segment matrix = Adafruit_7segment();
bool drawDots = true;

uint16_t time;
int mn, hr, dy, us, uc, yr, ly, ls, ds;

//                     1         2         3         4         5
//          012345678901234567890123456789012345678901234567890123456789
char L[] = ".abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";

uint8_t prevCode = CODE_N;
//                          _N  _U  _W  _P  _X
char printCode[CODE_X + 1] = {' ', '.', '-', '|', '*'};
uint8_t signalSegments[] = {
    /*0*/ ~(0x80),
    /*1*/ ~(0x08),
    /*2*/ ~(0x08),
    /*3*/ ~(0x08 | 0x04 | 0x10),
    /*4*/ ~(0x08 | 0x40),
    /*5*/ ~(0x08 | 0x40),
    /*6*/ ~(0x08 | 0x40 | 0x04 | 0x10),
    /*7*/ ~(0x08 | 0x40 | 0x01),
    /*8*/ ~(0x08 | 0x40 | 0x01),
    /*9*/ ~(0x08 | 0x40 | 0x01 | 0x04 | 0x10),
    /*10*/ ~(0x08 | 0x40 | 0x01 | 0x02 | 0x20),
};
int lastCycle = HIGH, prevSerial = 0;

void setup() {
    Serial.begin(115200);
    matrix.begin(0x70);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RADIO_POWERDOWN_PIN, OUTPUT);
    pinMode(RADIO_IN_PIN, INPUT);
    
    rtc.begin(RTC_SELECT_PIN);
    rtc.writeSQW(SQW_SQUARE_1); // 1Hz signal on RTC_INTERRUPT_PIN
    
    digitalWrite(RADIO_POWERDOWN_PIN, LOW);     // turn on radio
    radioPort = digitalPinToPort(RADIO_IN_PIN); // for optimized digitalRead
    radioBit = digitalPinToBitMask(RADIO_IN_PIN);
    avg = 0;
    xpast = 0;
    pr = EMPTY;
    wrap = 0;
    n = 0;
    // Start timer1 for periodic interrupt at SAMPLE_HZ per second
    noInterrupts();
    {
        TCCR1A = 0;
        TCCR1B = 0;
        TCNT1 = 0;
        OCR1A = F_CPU / 256 / SAMPLE_HZ - 1; // compare match register 16MHz / 256 prescaler / SAMPLE_HZ
                                             // https://github.com/ahooper/WWVBClock/issues/1
        TCCR1B |= (1 << WGM12);              // CTC mode
        TCCR1B |= (1 << CS12);               // 256 prescaler
        TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt
    }
    interrupts();
    Serial.println("Running AtomicClock");
}

void decodeAndSetTime(void) {
    // Decode frame parts
    /*
    decodeAndSetTime() is working with the table in FramePattern[], that
    provides the order of the components in the one-minute frame. It checks
    that the fixed markers (frame reference, end of frame, unweighted) come at
    the expected slots in the one-minute, and also accumulates the bits of the
    binary codes for the time and date decimal digit components into decode[].
    The various decimal digit components each have a unique index in decode[]
    defined by the enumeration FPxx. If the fixed markers don't match at the
    expected points, the decode fails by return and the real-time clock is not
    updated.

    decodeAndSetTime() is called from loop(), which is looking at the count of
    timer-interrupt signal samples over a one second interval to determine the
    length of the modulation using empirical thresholds. It saves the
    one-seconds counts in frame[] and calls decodeAndSetTime() when it sees two
    successive frame markers indicating the end of one frame and start of the next.
    */
    memset(decode, 0, sizeof decode);
    for (int i = 0; i < FRAME_SIZE; i++) {
        Serial.print((int)frame[i]);
        if (i < FRAME_SIZE) {
          Serial.print(", ");
        }
    }
    Serial.println();

    // check for valid frame values
    for (int x = 0; x < FRAME_SIZE; x++) {
        uint8_t p = FramePattern[x], c = frame[x];
        switch (p) {
            case FPEF: // End of frame same as FPRM
            case FPRM: // Frame reference marker: .8L+.2H
                if (c != CODE_P) {
                    return;
                }
                break;
            case FPUU: // Unweighted: .2L+.8H
                if (c != CODE_U) {
                    return;
                }
                break;
            default: // 1 = .5L+.5H / 0 = .2L+.8H
                // binary coding
                if (c == CODE_U)
                    decode[p] = (decode[p] << 1);
                else if (c == CODE_W)
                    decode[p] = (decode[p] << 1) | 1;
                else {
                    Serial.print("bad bit value at ");
                    Serial.println(x);
                    return;
                }
                break;
        }
    }

    // TODO: calc expected frame and compare.
    // Reset clock after 3 failed compares that validate with each other

    // Combine decimal digits
    mn = decode[FPM1] * 10 + decode[FPM2];
    hr = decode[FPH1] * 10 + decode[FPH2];
    dy = decode[FPD1] * 100 + decode[FPD2] * 10 + decode[FPD3];
    us = decode[FPUS];
    uc = decode[FPUC];
    yr = decode[FPY1] * 10 + decode[FPY2] + CENTURY;
    ly = decode[FPLY];
    ls = decode[FPLS];
    ds = decode[FPDS];
    if (Serial) {
        Serial.print("Datetime: ");
        Serial.print("Y");
        Serial.print(yr);
        Serial.print("D");
        Serial.print(dy);
        Serial.print("H");
        Serial.print(hr);
        Serial.print("M");
        Serial.print(mn);
        Serial.print("US");
        if (us == 5)
            Serial.write('+');
        else if (us == 2)
            Serial.write('-');
        else
            Serial.print(us);
        Serial.print("UC");
        Serial.print(uc);
        Serial.print("LY");
        Serial.print(ly);
        Serial.print("LS");
        Serial.print(ls);
        Serial.print("DS");
        Serial.print(ds);
        Serial.println();
    }
    // Correct for 1 minute coding delay from on-time point
    mn += 1;
    if (mn >= 60) {
        hr += 1;
        mn = 0;
        if (hr >= 24) {
            dy += 1;
            hr = 0;
            if (dy >= 365 + ly) {
                yr += 1;
                dy = 1;
            }
        }
    }
    uint8_t dst = 0;
#if DST_ENABLED
    // is dst not in effect(0), ending(1), beginning(2), or in effect(3)?
    if (ds == 0) {
        // not in effect
        dst = 0;
    } else if (ds == 1 && (hr + zoneHours) > 2) {
        // ending on this UTC day. apply at local time 0200
        dst = 0;
    } else if (ds == 2 && (hr + zoneHours) > 2) {
        // beginning on this UTC day. apply at local time 0200
        dst = 1;
    } else if (ds == 3) {
      // dst is in effect
      dst = 1;
    }
#endif
    // correct for timezone and daylight saving time
    hr += zoneHours + dst;
    if (hr >= 24) {
        // handle overflow
        dy += 1;
        hr -= 24;
        if (dy >= 365 + ly) {
            yr += 1;
            dy = 1;
        }
    } else if (hr < 0) {
        // handle underflow
        dy -= 1;
        hr += 24;
        if (dy < 0) {
            yr -= 1;
            dy = 365;
        }
    }
    int mo = 1, dim;
    while (1) {
        dim = daysInMonth[mo];
        if (mo == 2 && ly == 1)
            dim += 1;
        if (dy <= dim)
            break;
        dy -= dim;
        mo += 1;
    }

    // TODO: RTC should probably store UTC rather than local
    // Update crystal clock
    rtc.setSecond(1); // TODO correct for start of minute marker
    rtc.setMinute(mn);
    rtc.setHour(hr);
    rtc.setYear(yr - CENTURY);
    rtc.setMonth(mo);
    rtc.setDate(dy);
    timeSet = true;
    cyclesSinceTimeSet = 0;
}

void timeToDisplay() {
    // calc the time in HHMM format from the RTC data
    // TODO: make this a char* so that leading zeros get displayed
    uint8_t h = rtc.hour();
    uint8_t m = rtc.minute();
    time = h * 100 + m;
}

void writeToDisplay() {
    drawDots = !drawDots;
    matrix.print(time);
    matrix.drawColon(drawDots);
    matrix.setBrightness(0); // 0-15 with 15 as brightest
    matrix.writeDisplay();
}

void loop() {
    if (pr != EMPTY) {
        pr = EMPTY;
        if (dur) {
            uint8_t d = dur;
            dur = 0;
            if (d >= 68 * SAMPLE_HZ / 100 && d < 92 * SAMPLE_HZ / 100)
                code = CODE_P;
            else if (d >= 38 * SAMPLE_HZ / 100 && d < 62 * SAMPLE_HZ / 100)
                code = CODE_W;
            else if (d >= 8 * SAMPLE_HZ / 100 && d < 32 * SAMPLE_HZ / 100)
                code = CODE_U;
            else
                code = CODE_X; // radio is off or on at this point
            // show codes while waiting for lock
            static_assert(SAMPLE_HZ < sizeof L, "SAMPLE_HZ exceeds length of L");
            if (code == CODE_P && prevCode == CODE_P) {
                // once per minute
                if (frameIndex == FRAME_SIZE) {
                    Serial.println("full frame detected");
                    decodeAndSetTime();
                } else {
                    Serial.print("partial frame detected. frameindex: ");
                    Serial.println(frameIndex);
                }
                frameIndex = 0;
            }
            if (frameIndex < FRAME_SIZE)
                frame[frameIndex++] = code;
            prevCode = code;
            code = CODE_N;
        }
    }
    if (digitalRead(RTC_INTERRUPT_PIN) != lastCycle) {
        // 1 Hz signal from crystal clock
        lastCycle = digitalRead(RTC_INTERRUPT_PIN);
        if (lastCycle == LOW) {
            rtc.update();
            timeToDisplay();
            writeToDisplay();
        }
    }
}
