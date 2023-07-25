#include <EEPROM.h>
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <SparkFunDS3234RTC.h>

#define CENTURY 2000
#define DST_ENABLED 1
#define HR12MODE_ENABLED 1

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
const uint8_t FramePattern[] =
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

const int8_t zoneHours = -6; // Dallas time

uint8_t frame[FRAME_SIZE];
uint8_t frameIndex = 0;
uint16_t decode[FPEF]; // accumulated parts of frame
//                             JanFebMarAprMayJunJulAugSepOctNovDec
const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const uint16_t days_in_year = 365;

Adafruit_7segment matrix = Adafruit_7segment();

int8_t minute, hour, ly, ls, ds;
int16_t date, us, uc, year;

uint8_t prevCode = CODE_N;
bool lastCycle = 1;

/**
 * Address of the version data. This should never change between versions.
 */
#define EEPROM_ADDR_VERSION_DATE 0x0

/**
 * Your data should start after this location.
 * On gcc, __DATE_ is 11 chars long, __TIME__ is 8
 */
#define EEPROM_ADD_START_OF_DATA 0x13

/**
 * Determine if this is the first power-up after
 * fresh program was loaded into the flash.
 */
boolean is_initial_program_load()
{
    const String version_date = __DATE__ __TIME__;
    uint16_t len = version_date.length();
    boolean is_ipl = false;

    for (unsigned int i = 0; i < len; i++)
    {
        int addr = EEPROM_ADDR_VERSION_DATE + i;

        if (EEPROM.read(addr) != version_date[i])
        {
            EEPROM.write(addr, version_date[i]);
            is_ipl = true;
        }
    }
    return is_ipl;
}

void setup()
{
    Serial.begin(115200);
    matrix.begin(0x70);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RADIO_POWERDOWN_PIN, OUTPUT);
    pinMode(RADIO_IN_PIN, INPUT);

    rtc.begin(RTC_SELECT_PIN);
    if (is_initial_program_load())
    {
        rtc.autoTime();
    }
    if (rtc.is12Hour())
    {
        rtc.set24Hour(true);
    }
    rtc.writeSQW(SQW_SQUARE_1); // 1Hz signal on RTC_INTERRUPT_PIN

    digitalWrite(RADIO_POWERDOWN_PIN, LOW);     // turn on radio
    radioPort = digitalPinToPort(RADIO_IN_PIN); // for optimized digitalRead
    radioBit = digitalPinToBitMask(RADIO_IN_PIN);
    avg = 0;
    xpast = 0;
    pr = EMPTY;
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
    Serial.print("Options: ");
    Serial.print(DST_ENABLED);
    Serial.print(HR12MODE_ENABLED);
    Serial.println();
}

void decodeAndSetTime()
{
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
    memset(decode, 0, sizeof(decode));
    for (int i = 0; i < FRAME_SIZE; i++)
    {
        Serial.print((int)frame[i]);
        if (i < FRAME_SIZE - 1)
        {
            Serial.print(", ");
        }
    }
    Serial.println();

    // check for valid frame values
    for (int x = 0; x < FRAME_SIZE; x++)
    {
        uint8_t p = FramePattern[x], c = frame[x];
        switch (p)
        {
        case FPEF: // End of frame same as FPRM
        case FPRM: // Frame reference marker: .8L+.2H
            if (c != CODE_P)
            {
                return;
            }
            break;
        case FPUU: // Unweighted: .2L+.8H
            if (c != CODE_U)
            {
                return;
            }
            break;
        default: // 1 = .5L+.5H / 0 = .2L+.8H
            // binary coding
            if (c == CODE_U)
                decode[p] = (decode[p] << 1);
            else if (c == CODE_W)
                decode[p] = (decode[p] << 1) | 1;
            else
            {
                Serial.print("bad bit value at ");
                Serial.println(x);
                return;
            }
            break;
        }
    }
    Serial.println("good decode");

    // TODO: calc expected frame and compare.
    // Reset clock after 3 failed compares that validate with each other

    // Combine decimal digits
    minute = decode[FPM1] * 10 + decode[FPM2];
    hour = decode[FPH1] * 10 + decode[FPH2];
    date = decode[FPD1] * 100 + decode[FPD2] * 10 + decode[FPD3];
    us = decode[FPUS];
    uc = decode[FPUC];
    year = decode[FPY1] * 10 + decode[FPY2] + CENTURY;
    ly = decode[FPLY];
    ls = decode[FPLS];
    ds = decode[FPDS];
    if (Serial)
    {
        Serial.print("Datetime (UTC): ");
        Serial.print("Y");
        Serial.print(year);
        Serial.print("D");
        Serial.print(date);
        Serial.print("H");
        Serial.print(hour);
        Serial.print("M");
        Serial.print(minute);
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
    minute += 1;
    if (minute >= 60)
    {
        hour += 1;
        minute = 0;
        if (hour >= 24)
        {
            date += 1;
            hour = 0;
            if (date >= days_in_year + ly)
            {
                year += 1;
                date = 1;
            }
        }
    }
#if DST_ENABLED
    uint8_t dst = 0;
    // is dst not in effect(0), ending(1), beginning(2), or in effect(3)?
    if (ds == 0)
    {
        // not in effect
        dst = 0;
    }
    else if (ds == 1 && (hour + zoneHours) > 2)
    {
        // ending on this UTC day. apply at local time 0200
        dst = 0;
    }
    else if (ds == 2 && (hour + zoneHours) > 2)
    {
        // beginning on this UTC day. apply at local time 0200
        dst = 1;
    }
    else if (ds == 3)
    {
        // dst is in effect
        dst = 1;
    }
    // correct for timezone and daylight saving time
    hour += zoneHours + dst;
#else
    hr += zoneHours
#endif
    if (hour >= 24)
    {
        // handle overflow
        date += 1;
        hour -= 24;
        if (date >= days_in_year + ly)
        {
            year += 1;
            date = 1;
        }
    }
    else if (hour < 0)
    {
        // handle underflow
        date -= 1;
        hour += 24;
        if (date < 0)
        {
            year -= 1;
            date = 365;
        }
    }
    uint8_t month = 1, dim;
    while (1)
    {
        dim = daysInMonth[month - 1];
        if (month == 2 && ly == 1)
            dim += 1;
        if (date <= dim)
            break;
        date -= dim;
        month += 1;
    }

    // TODO: RTC should probably store UTC rather than local
    // Update crystal clock
    // Calculate weekday (from here: http://stackoverflow.com/a/21235587)
    // Result: 0 = Sunday, 6 = Saturday
    uint16_t wk_date = date;
    uint8_t weekday = (wk_date += month < 3 ? year-- : year - 2, 23 * month / 9 + wk_date + 4 + year / 4 - year / 100 + year / 400) % 7;
    weekday += 1; // Library defines Sunday=1, Saturday=7
    // 2 seconds for detecting the start of the minute by double markers (1 code plus 1 for...something that I forgot)
    rtc.setTime(2, minute, hour, weekday, date, month, year);

    if (Serial)
    {
        Serial.print("Datetime (Local): ");
        Serial.print(year);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(date);
        Serial.print(" ");
        Serial.print(hour);
        Serial.print(":");
        Serial.println(minute);
    }
    memset(decode, 0, sizeof(decode));
}

bool checkFrame()
{
    if (pr != EMPTY)
    {
        pr = EMPTY;
        if (dur)
        {
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
            if (code == CODE_P && prevCode == CODE_P)
            {
                // once per minute
                uint8_t temp = frameIndex;
                frameIndex = 0;
                if (temp == FRAME_SIZE)
                {
                    Serial.println("full frame detected");
                    return true;
                }
                else
                {
                    Serial.print("partial frame detected. frameindex: ");
                    Serial.println(temp);
                }
            }
            if (frameIndex == FRAME_SIZE)
                frameIndex = 0;
            frame[frameIndex++] = code;
            prevCode = code;
            code = CODE_N;
        }
    }
    return false;
}

uint8_t displayBrightness(uint8_t h, uint8_t m)
{
    // Change brightness based on local time
    // 0-15 with 15 as brightest
    const uint8_t rmp_up_t = 5;
    const uint8_t rmp_dwn_t = 20;
    const float rmp_rate = 15 / 60; // 1 hour rate
    uint8_t br = 0;
    if (h < rmp_up_t)
        br = 0;
    else if (h < rmp_up_t + 1)
    {
        // ramp up
        br = rmp_rate * m;
    }
    else if (h < rmp_dwn_t)
        br = 15;
    else if (h < rmp_dwn_t + 1)
    {
        // ramp down
        br = -rmp_rate * m + 15;
    }
    return br;
}

char *timeToDisplay(char *buffer, uint8_t h, uint8_t m)
{
#if HR12MODE_ENABLED
    sprintf(buffer, "%02d%02d", h > 12 ? h - 12 : h, m);
#else
    sprintf(buffer, "%02d%02d", h, m);
#endif
    return buffer;
}

void writeToDisplay(bool dots, uint8_t h, uint8_t m)
{
    char buffer[5];
    matrix.println(timeToDisplay(buffer, h, m));
    matrix.drawColon(dots);
    matrix.setBrightness(displayBrightness(h, m));
    matrix.writeDisplay();
}

void loop()
{
    if (checkFrame())
    {
        decodeAndSetTime();
    }
    // 1 Hz signal from RTC
    const bool currentCycle = digitalRead(RTC_INTERRUPT_PIN);
    if (currentCycle != lastCycle)
    {
        rtc.update();
        writeToDisplay(currentCycle, rtc.hour(), rtc.minute());
        lastCycle = currentCycle;
    }
}
