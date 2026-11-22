/* Perfume filling gun - Arduino Nano ATmega328P
   OLED I2C: SDA=A4, SCL=A5. U8g2 library required.
   Encoder: CLK=D3, DT=D4, SW=D5. Trigger NO=D6, COM=GND.
   Battery Voltage Divider (3 x 20kOhm = 1/3 tap): A0.
   MOSFET driver control=D9 (switches 9.6V buck-regulated supply to pump).
   Flow sensor on D2 removed — strictly time-based calibration & volume dispensing.
   Regulated 5V powers Nano and compatible peripherals; all grounds common.
   Animated boot logo comes from BootLogo.h (keep it in the sketch folder).

   --- ITERATION 14 (v14): ZERO-RUNOFF PRESET CUTOFF + 165.5mL/10s CAL + D10 BUZZER + 5-POINT PHYSICS ---
   1. Zero-Runoff Preset Dispensing & 165.5 mL / 10s Calibration Baseline (v14):
      - Aligns default calibratedTenthsML = 1655 (165.5 mL per 10.0s = 16.55 mL/s) and bumps EEPROM magic ('T','2')
        so stale 35.0 mL EEPROM values cannot cause 2.5-3s run-off on 8mL/10mL presets.
      - Adds 300ms post-start trigger-release bounce guard so mechanical trigger release or D6 EMI cannot falsely
        toggle FILLING <-> PAUSED mid-run, and unifies fillStartedAt = millis() before pumpOn().
      - Immediately forces D9 LOW (<1us) and transitions to COMPLETE inside servicePumpSafetyInline(), while
        suppressing new 8-page I2C OLED redraws during the final 180ms cutoff window.
   2. D10 Hardware Timer1 OC1B 5-Event F# Major Boot Audio + Active-Digit Flicker + 2.1KB Flash Optimization (v13):
      - Plays 5-event F# Major progression (70..2610ms: F#1 46Hz bass + F#4/A#4/C#5 triad) on D10 using direct
        Timer1 OC1B CTC hardware toggle (zero Tone.cpp overhead) + 16-bit utoa (~29,030 B / 94% Flash).
      - Flickers the active digit place (.1, 1, 10, 100 mL) on CAL_RESULT when cycling step size via Trigger.
   3. 5-Point Battery & Motor Physics Hardening + 3S Voltage Display (A0 x 3) (v12):
      - Pump-run freeze + 600ms post-stop OCV settle window, 10.20V/10.80V lockout hysteresis with open-A0 bypass,
        30ms hardware PWM soft-start on D9, and pre-sample capacitive discharge + 85/15 EMA filter.
*/
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "BootLogo.h"

// Early hardware reset hook (.init3): immediately disables watchdog after reset and pulls D9 LOW
// in <1 microsecond so the MOSFET gate is guaranteed OFF before setup() even begins.
void earlyHardwareSafeInit() __attribute__((naked)) __attribute__((section(".init3")));
void earlyHardwareSafeInit() {
  MCUSR = 0;
  wdt_disable();
  DDRB |= _BV(DDB1);     // D9 (PB1) configured as OUTPUT
  PORTB &= ~_BV(PORTB1); // D9 (PB1) forced LOW immediately
}

#define OLED_SH1106 1
#if OLED_SH1106
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#endif

const uint8_t OLED_ADDRESS = 0x3C;
const uint8_t ENC_A = 3, ENC_B = 4, ENC_SW = 5, TRIGGER = 6;
const uint8_t PUMP_PIN = 9;
const uint8_t BUZZER_PIN = 10;  // D10: Passive buzzer (+) to D10, (-) to GND
const uint8_t BATTERY_PIN = A0; // Solder 1/3 tap of 3 x 20kOhm battery divider to A0

// 3S Li-Ion Battery Monitoring via 3 x 20kOhm voltage divider (40kOhm top, 20kOhm bottom -> ratio = 3.0)
// 9.60V = 0% (3.20V at A0), 12.60V = 100% (4.20V at A0)
const uint16_t BATTERY_EMPTY_CENTI_V = 960U;  // 9.60V (0% bar scale floor)
const uint16_t BATTERY_FULL_CENTI_V  = 1260U; // 12.60V (100% bar scale ceiling)

// Dual-threshold low-battery hysteresis (Issue 2):
// Lock pump at <= 10.20V (3.40V at A0); unlock only when recovered >= 10.80V (3.60V at A0)
const uint16_t BATT_CUTOFF_CENTI_V  = 1020U; // 10.20V
const uint16_t BATT_RECOVER_CENTI_V = 1080U; // 10.80V

// Post-stop OCV rebound window (Issue 1)
const unsigned long PUMP_OCV_SETTLE_MS = 600UL;

// Time-based calibration: volume in tenths of a mL dispensed during a fixed 10.0-second run.
// Default 1655 = 165.5 mL per 10.0s (16.55 mL/s, matching measured 165-166 mL in 10s). Range: 1 (0.1 mL) to 5000 (500.0 mL).
uint16_t calibratedTenthsML = 1655;
const int CAL_EEPROM_MAGIC_ADDR = 24;
const int CAL_EEPROM_VALUE_ADDR = 26;
const uint8_t CAL_EEPROM_MAGIC_1 = 0x54; // 'T' (Time-based calibration magic byte 1)
const uint8_t CAL_EEPROM_MAGIC_2 = 0x32; // '2' (v12.2 magic byte resets stale 35.0mL EEPROM to 165.5mL default)

// Fixed 10.0-second duration for timed flow calibration
const unsigned long CAL_FIXED_DURATION_MS = 10000UL;
// Fast completion screen hold time (350 ms instead of 1500 ms)
const unsigned long COMPLETE_HOLD_MS = 350UL;
const unsigned long STOPPED_HOLD_MS = 1200UL;

const int8_t ENCODER_DIRECTION = -1;
const int8_t COUNTS_PER_DETENT = 4;
const uint16_t MIN_ML = 1, MAX_ML = 500;
const uint8_t MAX_PRESETS = 8;
uint16_t PRESETS[MAX_PRESETS] = {8, 10, 20, 50, 100};
uint8_t PRESET_COUNT = 5;
uint8_t brightness = 70, editBrightness = 70;
uint8_t settingsSelected = 0;
const char emptyNotice[] PROGMEM = "";
const char *notice = emptyNotice;
unsigned long noticeUntil = 0;
const uint8_t SAVE_SIZE = 22;

uint8_t configCRC(const uint8_t *data, uint8_t count) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < count; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                         : (uint8_t)(crc << 1);
  }
  return crc;
}

void saveSettings() {
  wdt_reset();
  uint8_t data[SAVE_SIZE] = {0x50, 0x47, 2, brightness, PRESET_COUNT};
  for (uint8_t i = 0; i < MAX_PRESETS; i++) {
    data[5 + i * 2] = PRESETS[i] & 0xFF;
    data[6 + i * 2] = PRESETS[i] >> 8;
  }
  data[21] = configCRC(data, 21);
  EEPROM.update(0, 0);
  for (uint8_t i = 1; i < SAVE_SIZE; i++) EEPROM.update(i, data[i]);
  EEPROM.update(0, data[0]);
  wdt_reset();
}

void loadSettings() {
  uint8_t data[SAVE_SIZE];
  for (uint8_t i = 0; i < SAVE_SIZE; i++) data[i] = EEPROM.read(i);

  if (data[0] != 0x50 || data[1] != 0x47 ||
      (data[2] != 1 && data[2] != 2) ||
      data[3] < 10 || data[3] > 100 ||
      data[4] < 1 || data[4] > MAX_PRESETS ||
      data[21] != configCRC(data, 21)) return;

  if (data[2] == 1) {
    brightness = data[3];
    saveSettings();
    return;
  }

  for (uint8_t i = 0; i < data[4]; i++) {
    uint16_t value = (uint16_t)data[5 + i * 2] |
                     ((uint16_t)data[6 + i * 2] << 8);
    if (value < MIN_ML || value > MAX_ML) return;
    if (i && value <= ((uint16_t)data[3 + i * 2] |
                       ((uint16_t)data[4 + i * 2] << 8))) return;
  }

  brightness = data[3];
  PRESET_COUNT = data[4];
  for (uint8_t i = 0; i < MAX_PRESETS; i++)
    PRESETS[i] = (uint16_t)data[5 + i * 2] |
                 ((uint16_t)data[6 + i * 2] << 8);
}

const char strLowBatt[] PROGMEM = "LOW BATT <10.2V! Charge";

void saveTimeCalibration() {
  wdt_reset();
  EEPROM.update(CAL_EEPROM_MAGIC_ADDR, 0);
  EEPROM.update(CAL_EEPROM_MAGIC_ADDR + 1, 0);
  EEPROM.update(CAL_EEPROM_VALUE_ADDR, (uint8_t)(calibratedTenthsML & 0xFF));
  EEPROM.update(CAL_EEPROM_VALUE_ADDR + 1, (uint8_t)(calibratedTenthsML >> 8));
  EEPROM.update(CAL_EEPROM_MAGIC_ADDR, CAL_EEPROM_MAGIC_1);
  EEPROM.update(CAL_EEPROM_MAGIC_ADDR + 1, CAL_EEPROM_MAGIC_2);
  wdt_reset();
}

void loadTimeCalibration() {
  if (EEPROM.read(CAL_EEPROM_MAGIC_ADDR) != CAL_EEPROM_MAGIC_1 ||
      EEPROM.read(CAL_EEPROM_MAGIC_ADDR + 1) != CAL_EEPROM_MAGIC_2) {
    return;
  }

  uint16_t savedTenths = (uint16_t)EEPROM.read(CAL_EEPROM_VALUE_ADDR) |
                         ((uint16_t)EEPROM.read(CAL_EEPROM_VALUE_ADDR + 1) << 8);

  if (savedTenths >= 1U && savedTenths <= (uint16_t)(MAX_ML * 10U)) {
    calibratedTenthsML = savedTenths;
  }
}

enum Screen : uint8_t {
  MENU, CUSTOM, READY, FILLING, PAUSED, COMPLETE, STOPPED,
  SETTINGS, BATTERY_INFO, BRIGHTNESS, ADD_PRESET, ABOUT, RESET_PRESETS,
  CALIBRATION, CALIBRATING, CAL_RESULT, MANUAL_DISPENSE, FLUSH_MODE
};

void finishFill(Screen result, unsigned long now);
void servicePumpSafetyInline();

Screen screen = MENU;
uint8_t selected = 0;
uint16_t targetML = 10, customML = 45, editML = 45;
// Measured calibration volume in tenths of mL (default 1655 = 165.5 mL per 10s, range 0.1 - 500.0 mL)
uint16_t calibrationTenthsML = 1655;
// Knob turn step tier on CAL_RESULT: 0 = 0.1 mL, 1 = 1 mL, 2 = 10 mL, 3 = 100 mL (cycled via Trigger)
uint8_t calStepTier = 0;
uint32_t deliveredUL = 0;
unsigned long fillStartedAt = 0;
unsigned long accumulatedFillMs = 0;
unsigned long targetDurationMs = 0;
unsigned long resultAt = 0, lastFrame = 0;
unsigned long flushStartedAt = 0;
unsigned long pumpStoppedAt = 0;
bool flushRunning = false;
bool pumpRunningState = false;
bool i2cSuspendedDuringPump = false;
bool lowBatteryLocked = false;
const unsigned long FLUSH_DURATION_MS = 1200UL;
unsigned long lastTurn = 0, lastBattery = 0;

// Live A0 ADC & Battery state (integer centi-volts: rawPinCentiV = 1/3 tap, packCentiV = A0 x 3 original 3S voltage)
uint16_t rawAdcCount = 0;
uint16_t rawPinCentiV = 0;
uint16_t packCentiV = 0;
int8_t batteryPercent = -1; // -1 = pin open / disconnected ("?")

bool ignoreStartTrigger = false;
const unsigned long TRIGGER_HOLD_MS = 1000;
const unsigned long BOOT_DURATION_MS = 2650; // Covers all 5 boot audio events through ~2610 ms
unsigned long bootStartedAt = 0;
bool bootActive = true;
bool bootAudioDone = false;
uint16_t lastBuzzerTop = 0;

volatile int16_t encoderTicks = 0;
volatile uint8_t encoderPrevious = 0;

void unwedgeI2CBus();
void configureI2CSafety();
void reinitI2CAndOLED();
void recoverI2CAndOLEDIfGlitched();

// Boot Buzzer Audio Schedule on D10 (Hardware Timer1 OC1B CTC toggle — zero Tone.cpp library overhead):
// Event  Start(ms)  Dur(ms)  Bass(orig)     Chord tones
// 1      70         370      F#1 (46 Hz)    F#4 (370 Hz)  A#4 (466 Hz)  C#5 (554 Hz)
// 2      440        670      F#1 (46 Hz)    F#4 (370 Hz)  A#4 (466 Hz)  C#5 (554 Hz)
// 3      1110       650      F#1 (46 Hz)    F#4 (370 Hz)  A#4 (466 Hz)  C#5 (554 Hz)
// 4      1760       420      F#1 (46 Hz)    F#4 (370 Hz)  A#4 (466 Hz)  C#5 (554 Hz)
// 5      2180       430      F#1 (46 Hz)    F#4 (370 Hz)  A#4 (466 Hz)  C#5 (554 Hz) (ends ~2610 ms)
// Precomputed Timer1 CTC OCR1A values at /64 prescaler (f = 125000 / (1 + OCR1A)):
// F#1 (46 Hz) -> 2716, F#4 (370 Hz) -> 337, A#4 (466 Hz) -> 267, C#5 (554 Hz) -> 225
const uint16_t bootEventStarts[5] PROGMEM = {  70, 440, 1110, 1760, 2180 };
const uint16_t bootEventDurs[5]   PROGMEM = { 370, 670,  650,  420,  430 };
const uint16_t BOOT_TOP_FS1 = 2716U; // F#1 (46.01 Hz)
const uint16_t bootChordTops[3] PROGMEM = {
  337U, // F#4 (369.82 Hz)
  267U, // A#4 (466.42 Hz)
  225U  // C#5 (553.10 Hz)
};

void stopBootBuzzer() {
  if (lastBuzzerTop != 0) {
    TCCR1A = _BV(WGM10);            // Restore standard Arduino Timer1 8-bit PWM mode for D9 analogWrite()
    TCCR1B = _BV(CS11) | _BV(CS10); // Restore /64 prescaler
    PORTB &= ~_BV(PORTB2);          // Drive D10 (PB2) LOW
    lastBuzzerTop = 0;
  }
}

void updateBootBuzzer(unsigned long elapsed) {
  if (bootAudioDone) return;

  if (elapsed >= 2610UL) {
    stopBootBuzzer();
    bootAudioDone = true;
    return;
  }

  uint16_t targetTop = 0;
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t startMs = pgm_read_word(&bootEventStarts[i]);
    uint16_t durMs   = pgm_read_word(&bootEventDurs[i]);
    if (elapsed >= startMs && elapsed < (unsigned long)(startMs + durMs)) {
      uint16_t relMs = (uint16_t)(elapsed - startMs);
      // 18ms articulation release gap at the end of each event so the 5 chord hits are distinct
      if (relMs + 18U < durMs) {
        if (relMs < 45U) {
          targetTop = BOOT_TOP_FS1; // Initial F#1 (46 Hz) bass strike
        } else {
          // Rapidly arpeggiate F#4 -> A#4 -> C#5 (28ms per chord tone)
          uint8_t chordIdx = (uint8_t)(((relMs - 45U) / 28U) % 3U);
          targetTop = pgm_read_word(&bootChordTops[chordIdx]);
        }
      }
      break;
    }
  }

  if (targetTop == 0) {
    stopBootBuzzer();
  } else if (targetTop != lastBuzzerTop) {
    OCR1A = targetTop;
    OCR1B = 0;
    if (TCNT1 >= targetTop) TCNT1 = 0;
    TCCR1A = _BV(COM1B0);                        // Hardware toggle OC1B (D10); D9 (OC1A) stays normal GPIO LOW
    TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10); // CTC mode (TOP = OCR1A), /64 prescaler
    lastBuzzerTop = targetTop;
  }
}

// Convert active pump run time (ms) into delivered microliters (uL) using 32-bit integer math
// (calibratedTenthsML tenths of mL in 10,000 ms = calibratedTenthsML * 100 uL in 10,000 ms)
uint32_t msToDeliveredUL(unsigned long elapsedMs) {
  uint32_t cal = (uint32_t)calibratedTenthsML;
  return (elapsedMs / 100UL) * cal + ((elapsedMs % 100UL) * cal) / 100UL;
}

// Compute exact target pump duration (ms) for a desired volume in mL using identical linear timing to 10s calibration
unsigned long calcTargetDurationMs(uint16_t volumeML) {
  if (calibratedTenthsML == 0) return 0;
  uint32_t num = (uint32_t)volumeML * 100000UL + (uint32_t)(calibratedTenthsML / 2U);
  return (unsigned long)(num / (uint32_t)calibratedTenthsML);
}

// Issue 3 Fix: 30ms Hardware PWM Soft-Start on D9 (eliminates motor zero-BEMF dead-short inrush spike)
void pumpOn() {
  if (lowBatteryLocked) return;
  if (!pumpRunningState) {
    pumpRunningState = true;
    for (uint16_t pwm = 60; pwm <= 220; pwm += 40) {
      wdt_reset();
      analogWrite(PUMP_PIN, (uint8_t)pwm);
      delay(5);
    }
    digitalWrite(PUMP_PIN, HIGH);
    delay(5);
    lastFrame = millis();
  }
}

void pumpOff() {
  digitalWrite(PUMP_PIN, LOW);
  if (pumpRunningState) {
    pumpRunningState = false;
    pumpStoppedAt = millis(); // Start 600ms post-stop OCV recovery window (Issue 1)
    wdt_reset();
    delay(15); // Allow inductive flyback current to decay before resuming I2C
    if (i2cSuspendedDuringPump) {
      i2cSuspendedDuringPump = false;
      reinitI2CAndOLED();
    }
    lastFrame = 0; // Trigger immediate clean screen update now that pump is OFF
  }
}

// Issues 1, 2 & 4 Fix:
// - Freezes during pump run AND for 600ms after pumpOff() (OCV rebound window)
// - Drains floating charge on A0 so an unconnected pin reads 0.00V ("?") and bypasses lockout
// - Multiplies 1/3 A0 reading by 3 to show original 3S voltage, seeds immediately on wire connect,
//   and filters with 85% / 15% EMA while enforcing 10.20V cutoff / 10.80V recovery hysteresis.
void updateBattery(unsigned long now, bool forceSample = false) {
  if (pumpRunningState && !forceSample) return;
  if (!forceSample && pumpStoppedAt != 0 && (now - pumpStoppedAt < PUMP_OCV_SETTLE_MS)) return;
  if (!forceSample && (now - lastBattery < 150UL)) return;
  lastBattery = now;

  // 1. Drain stray floating charge on A0 (PC0) pin capacitance (~15pF) for 3us
  DDRC |= _BV(DDC0);
  PORTC &= ~_BV(PORTC0);
  delayMicroseconds(3);
  DDRC &= ~_BV(DDC0);
  PORTC &= ~_BV(PORTC0);

  // 2. Discharge internal ADC sample-and-hold capacitor via internal GND (0x0F) mux channel
  ADMUX = (DEFAULT << 6) | 0x0F;
  delayMicroseconds(50);

  // 3. Wait 250us and average 4 ADC readings
  delayMicroseconds(250);
  (void)analogRead(BATTERY_PIN);
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    sum += (uint16_t)analogRead(BATTERY_PIN);
  }
  uint16_t adc = (sum + 2U) >> 2;

  if (adc < 25U) adc = 0;
  rawAdcCount = adc;

  // Reconstruct original 3S battery voltage (A0 x 3 in centi-volts: 0..1500 -> 0.00V..15.00V)
  uint16_t samplePackCentiV = (uint16_t)(((uint32_t)adc * 1500UL + 511UL) / 1023UL);
  rawPinCentiV = (samplePackCentiV + 1U) / 3U;

  // If A0 is below 0.50V (ADC < 102), wire is open: show 0.00V ("?") and bypass lockout
  if (adc < 102U) {
    packCentiV = 0;
    batteryPercent = -1;
    lowBatteryLocked = false;
    return;
  }

  // Issue 4: Seed immediately when wire is first connected, then apply 85% / 15% EMA filter
  if (batteryPercent < 0 || packCentiV < 150U || forceSample) {
    packCentiV = samplePackCentiV;
  } else {
    packCentiV = (uint16_t)(((uint32_t)packCentiV * 85UL + (uint32_t)samplePackCentiV * 15UL + 50UL) / 100UL);
  }

  // Issue 2: Dual-Threshold Hysteresis (10.20V Cutoff / 10.80V Recovery)
  if (packCentiV <= BATT_CUTOFF_CENTI_V) {
    lowBatteryLocked = true;
  } else if (packCentiV >= BATT_RECOVER_CENTI_V) {
    lowBatteryLocked = false;
  }

  if (packCentiV <= BATTERY_EMPTY_CENTI_V) {
    batteryPercent = 0;
  } else if (packCentiV >= BATTERY_FULL_CENTI_V) {
    batteryPercent = 100;
  } else {
    batteryPercent = (int8_t)(((uint16_t)(packCentiV - BATTERY_EMPTY_CENTI_V) + 1U) / 3U);
  }
}

ISR(PCINT2_vect) {
  static const int8_t table[16] PROGMEM = {
    0, -1, 1, 0, 1, 0, 0, -1,
    -1, 0, 0, 1, 0, 1, -1, 0
  };
  uint8_t now = (PIND >> 3) & 3;
  int8_t delta = (int8_t)pgm_read_byte(&table[(encoderPrevious << 2) | now]);
  encoderPrevious = now;
  if (encoderTicks > -2000 && encoderTicks < 2000)
    encoderTicks += delta;
}

struct Button {
  uint8_t pin, debounceMs;
  bool rawDown, down, armed, longSent;
  bool pressed, clicked, held;
  unsigned long changedAt, downAt;

  void begin(uint8_t p, uint8_t debounce) {
    pin = p;
    debounceMs = debounce;
    pinMode(pin, INPUT_PULLUP);
    rawDown = down = (digitalRead(pin) == LOW);
    armed = !down;
    pressed = clicked = held = longSent = false;
    changedAt = downAt = millis();
  }

  void update(unsigned long now) {
    pressed = clicked = held = false;
    bool reading = digitalRead(pin) == LOW;

    if (reading != rawDown) {
      rawDown = reading;
      changedAt = now;
    }

    if (now - changedAt >= debounceMs && down != rawDown) {
      down = rawDown;
      if (down) {
        downAt = now;
        longSent = false;
        if (armed) pressed = true;
      } else {
        if (armed && !longSent) clicked = true;
        armed = true;
      }
    }

    if (pin == TRIGGER && down && armed && !longSent &&
        now - downAt >= TRIGGER_HOLD_MS) {
      held = true;
      longSent = true;
    }
  }
};

Button knob, trigger;

int16_t readEncoder() {
  static int16_t remainder = 0;
  int16_t ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ticks = encoderTicks;
    encoderTicks = 0;
  }
  remainder += ticks;
  int16_t detents = remainder / COUNTS_PER_DETENT;
  remainder %= COUNTS_PER_DETENT;
  return detents * ENCODER_DIRECTION;
}

// NXP I2C specification 9-clock bus un-wedger: forces SH1106 slave to release SDA if stuck LOW mid-byte
void unwedgeI2CBus() {
  TWCR = 0;
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(A5, LOW);
    delayMicroseconds(10);
    digitalWrite(A5, HIGH);
    delayMicroseconds(10);
    if (digitalRead(A4) == HIGH) break;
  }
  pinMode(A4, OUTPUT);
  digitalWrite(A4, LOW);
  delayMicroseconds(10);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  pinMode(A4, INPUT_PULLUP);
  delayMicroseconds(10);
}

// Configure I2C bus timeout so Wire.h can never loop infinitely on motor noise
void configureI2CSafety() {
  Wire.setClock(100000UL);
  Wire.setWireTimeout(2500, true); // 2500us timeout + automatic bus reset
  Wire.clearWireTimeoutFlag();
}

void reinitI2CAndOLED() {
  unwedgeI2CBus();
  Wire.begin();
  configureI2CSafety();
  Wire.beginTransmission(OLED_ADDRESS);
  Wire.write(0x00);
  Wire.write(0xAF);
  Wire.write(0xA4);
  Wire.write(0xA6);
  Wire.write(0x81);
  Wire.write((uint8_t)((uint16_t)brightness * 255 / 100));
  Wire.endTransmission();
  Wire.clearWireTimeoutFlag();
}

// Non-blanking I2C & SH1106 recovery with Active-Pump Circuit Breaker
void recoverI2CAndOLEDIfGlitched() {
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    servicePumpSafetyInline();
    if (pumpRunningState) {
      i2cSuspendedDuringPump = true;
      return;
    }
    reinitI2CAndOLED();
  }
}

// Inline time-based pump cutoff check executed between every OLED I2C page transfer
void servicePumpSafetyInline() {
  wdt_reset();
  unsigned long now = millis();
  if (screen == FILLING && pumpRunningState) {
    if (accumulatedFillMs + (now - fillStartedAt) >= targetDurationMs) {
      PORTB &= ~_BV(PORTB1); // Force D9 LOW in <1us
      deliveredUL = (uint32_t)targetML * 1000UL;
      finishFill(COMPLETE, now);
    }
  } else if (screen == FLUSH_MODE && flushRunning) {
    if (now - flushStartedAt >= FLUSH_DURATION_MS) {
      PORTB &= ~_BV(PORTB1);
      pumpOff();
      flushRunning = false;
    }
  } else if (screen == CALIBRATING && pumpRunningState) {
    if (now - fillStartedAt >= CAL_FIXED_DURATION_MS) {
      PORTB &= ~_BV(PORTB1);
      pumpOff();
    }
  }
}

bool checkBatteryLockout(unsigned long now) {
  if (lowBatteryLocked) {
    notice = strLowBatt;
    noticeUntil = now + 2200UL;
    return true;
  }
  return false;
}

void startFill(unsigned long now) {
  if (checkBatteryLockout(now)) return;
  pumpOff();
  deliveredUL = 0;
  accumulatedFillMs = 0;
  targetDurationMs = calcTargetDurationMs(targetML);
  if (targetDurationMs == 0) targetDurationMs = 1;
  screen = FILLING;
  ignoreStartTrigger = true;
  fillStartedAt = millis();
  pumpOn();
}

void finishFill(Screen result, unsigned long now) {
  PORTB &= ~_BV(PORTB1); // Guarantee D9 LOW immediately
  pumpOff();
  screen = result;
  resultAt = now;
}

void startManualCalibration(unsigned long now) {
  if (checkBatteryLockout(now)) return;
  pumpOff();
  screen = CALIBRATING;
  fillStartedAt = millis();
  pumpOn();
}

void finishManualCalibration(unsigned long now) {
  pumpOff();
  calibrationTenthsML = calibratedTenthsML;
  calStepTier = 0; // Default knob turn step = 0.1 mL
  lastTurn = 0;
  screen = CAL_RESULT;
  resultAt = now;
}

// Single static scratch buffer eliminates repeated stack allocations
static char progmemScratch[30];

// Formats centi-volts (e.g. 385 -> "3.85V", 1155 -> "11.55V") using 16-bit utoa
void formatCentiV(uint16_t centiV, char *out) {
  utoa(centiV / 100U, out, 10);
  uint8_t len = strlen(out);
  uint8_t rem = (uint8_t)(centiV % 100U);
  out[len] = '.';
  out[len + 1] = (char)('0' + (rem / 10U));
  out[len + 2] = (char)('0' + (rem % 10U));
  out[len + 3] = 'V';
  out[len + 4] = '\0';
}

void copyFlashToScratch(const char *s) {
  strncpy_P(progmemScratch, s, sizeof(progmemScratch) - 1);
  progmemScratch[sizeof(progmemScratch) - 1] = '\0';
}

void drawFlashStr(int x, int y, const char *s) {
  copyFlashToScratch(s);
  oled.drawStr(x, y, progmemScratch);
}

void centeredRAM(const char *s, uint8_t baseline) {
  oled.drawStr((128 - oled.getStrWidth(s)) / 2, baseline, s);
}

void centered(const char *s, uint8_t baseline) {
  copyFlashToScratch(s);
  centeredRAM(progmemScratch, baseline);
}

void drawHeader(const char *title) {
  oled.setFont(u8g2_font_5x7_tf);
  drawFlashStr(0, 8, title);

  // Format original 3S battery voltage (A0 reading multiplied by 3, e.g. "11.55V" or "0.00V")
  char vBuf[10];
  formatCentiV(packCentiV, vBuf);
  int vX = 105 - oled.getStrWidth(vBuf);

  // Clear status area on top-right so long header titles never overlap voltage + 4-bar icon
  oled.setDrawColor(0);
  oled.drawBox(vX - 2, 0, 128 - (vX - 2), 10);
  oled.setDrawColor(1);

  // Draw multiplied-by-3 original 3S Battery Voltage right next to the 4-bar icon
  oled.drawStr(vX, 8, vBuf);

  // Draw 4-Bar Battery Icon at x=108..127, y=1..8
  oled.drawFrame(108, 1, 18, 8);
  oled.drawBox(126, 3, 2, 4);

  if (batteryPercent < 0) {
    drawFlashStr(115, 8, PSTR("?"));
  } else if (lowBatteryLocked) {
    drawFlashStr(115, 8, PSTR("!"));
  } else {
    uint8_t bars = batteryPercent == 0 ? 0 : (uint8_t)((batteryPercent + 24) / 25);
    for (uint8_t i = 0; i < bars; i++)
      oled.drawBox(110 + i * 4, 3, 3, 4);
  }

  oled.drawHLine(0, 11, 128);
}

void footer(const char *text) {
  oled.setFont(u8g2_font_5x7_tf);
  centered(text, 63);
}

void drawVolumeStr(const char *number, uint8_t baseline) {
  oled.setFont(u8g2_font_helvB24_tn);
  int numberWidth = oled.getStrWidth(number);
  int x = (128 - numberWidth - 17) / 2;
  oled.drawStr(x, baseline, number);
  oled.setFont(u8g2_font_6x10_tf);
  drawFlashStr(x + numberWidth + 5, baseline, PSTR("mL"));
}

void drawVolume(uint16_t volume, uint8_t baseline) {
  char number[8];
  utoa(volume, number, 10);
  drawVolumeStr(number, baseline);
}

// Draws volume with 0.1 mL precision (e.g. "35.0 mL" or "035.0 mL") and flickers the digit
// corresponding to the active step size tier (0 = 0.1 mL, 1 = 1 mL, 2 = 10 mL, 3 = 100 mL)
void drawVolumeTenths(uint16_t tenths, uint8_t baseline, uint8_t stepTier) {
  char number[10];
  uint16_t whole = tenths / 10U;
  uint8_t pos = 0;

  // Pad with leading '0' if the selected step place (10s or 100s) is higher than the current value
  if (stepTier >= 3 && whole < 100U) number[pos++] = '0';
  if (stepTier >= 2 && whole < 10U)  number[pos++] = '0';

  utoa(whole, number + pos, 10);
  uint8_t len = strlen(number);
  number[len] = '.';
  number[len + 1] = (char)('0' + (tenths % 10U));
  number[len + 2] = '\0';
  uint8_t totalLen = len + 2;

  oled.setFont(u8g2_font_helvB24_tn);
  int numberWidth = oled.getStrWidth(number);
  int x = (128 - numberWidth - 17) / 2;
  oled.drawStr(x, baseline, number);

  int8_t blinkIdx = (stepTier == 0) ? (int8_t)(totalLen - 1) : (int8_t)len - (int8_t)stepTier;

  // Flicker every 256ms (~2 Hz); stay solid ON while actively turning the knob
  bool activelyTurning = (lastTurn != 0) && ((millis() - lastTurn) < 350UL);
  bool blinkOff = !activelyTurning && ((millis() & 256UL) == 0UL);

  if (blinkOff && blinkIdx >= 0 && blinkIdx < (int8_t)totalLen) {
    char singleChar[2] = { number[blinkIdx], '\0' };
    int digitW = oled.getStrWidth(singleChar);
    int digitX = x + numberWidth - oled.getStrWidth(number + blinkIdx);

    oled.setDrawColor(0);
    oled.drawBox(digitX - 1, baseline - 25, digitW + 2, 26);
    oled.setDrawColor(1);
    oled.drawBox(digitX, baseline - 1, digitW, 2);
  }

  oled.setFont(u8g2_font_6x10_tf);
  drawFlashStr(x + numberWidth + 5, baseline, PSTR("mL"));
}

void drawScreen() {
  char text[28];

  const char *title =
    screen == MENU ? PSTR("FILL VOLUME") :
    screen == CUSTOM ? PSTR("CUSTOM") :
    screen == SETTINGS ? PSTR("SETTINGS") :
    screen == BATTERY_INFO ? PSTR("BATTERY") :
    screen == BRIGHTNESS ? PSTR("BRIGHTNESS") :
    screen == ADD_PRESET ? PSTR("ADD PRESET") :
    screen == ABOUT ? PSTR("ABOUT") :
    screen == RESET_PRESETS ? PSTR("RESET PRESETS") :
    screen == CALIBRATION ? PSTR("FLOW CALIB") :
    screen == CALIBRATING ? PSTR("CALIBRATING") :
    screen == CAL_RESULT ? PSTR("CAL RESULT") :
    screen == MANUAL_DISPENSE ? PSTR("MANUAL") :
    screen == FLUSH_MODE ? PSTR("FLUSH MODE") :
    screen == READY ? PSTR("READY") :
    screen == FILLING ? PSTR("FILLING") :
    screen == PAUSED ? PSTR("PAUSED") :
    screen == COMPLETE ? PSTR("DONE") : PSTR("STOPPED");

  if (pumpRunningState && i2cSuspendedDuringPump) {
    servicePumpSafetyInline();
    return;
  }

  oled.firstPage();
  do {
    servicePumpSafetyInline();
    if (Wire.getWireTimeoutFlag()) break;

    if (screen != ABOUT) drawHeader(title);

    if (screen == SETTINGS) {
      const char *items[] = {
        PSTR("Battery voltage"), PSTR("Brightness"), PSTR("Add preset"),
        PSTR("Reset presets"), PSTR("About"), PSTR("Back")
      };
      uint8_t first = constrain((int)settingsSelected - 1, 0, 3);
      oled.setFont(u8g2_font_6x10_tf);

      for (uint8_t row = 0; row < 3; row++) {
        uint8_t index = first + row;
        uint8_t y = 14 + row * 13;
        if (index == settingsSelected) {
          oled.drawRBox(2, y, 124, 12, 2);
          oled.setDrawColor(0);
        }
        drawFlashStr(9, y + 9, items[index]);
        oled.setDrawColor(1);
      }
      footer(pgm_read_byte(notice) ? notice : PSTR("Turn:Browse Press:Open"));

    } else if (screen == BATTERY_INFO) {
      char vStr[10];
      oled.setFont(u8g2_font_5x7_tf);

      // Row 1: Original 3S Battery Voltage (A0 x 3) + Percentage
      strcpy_P(text, PSTR("3S Batt: "));
      formatCentiV(packCentiV, vStr);
      strcat(text, vStr);
      strcat_P(text, PSTR(" ("));
      if (batteryPercent < 0) {
        strcat_P(text, PSTR("--%)"));
      } else {
        utoa((uint16_t)batteryPercent, vStr, 10);
        strcat(text, vStr);
        strcat_P(text, PSTR("%)"));
      }
      oled.drawStr(2, 23, text);

      // Row 2: 1/3 Divider Voltage at A0 + Raw ADC Count (0-1023)
      strcpy_P(text, PSTR("A0 (1/3): "));
      formatCentiV(rawPinCentiV, vStr);
      strcat(text, vStr);
      strcat_P(text, PSTR(" ADC:"));
      utoa(rawAdcCount, vStr, 10);
      strcat(text, vStr);
      oled.drawStr(2, 35, text);

      // Row 3: Live Hardware Connection & Hysteresis Lockout Status
      if (rawAdcCount < 102U) {
        drawFlashStr(2, 48, PSTR("Status: A0 OPEN (BYPASS)"));
      } else if (lowBatteryLocked) {
        drawFlashStr(2, 48, PSTR("Status: LOCKED <10.2V! CHG"));
      } else if (rawAdcCount > 895U) {
        drawFlashStr(2, 48, PSTR("Status: >4.37V! CHECK TAP"));
      } else {
        drawFlashStr(2, 48, PSTR("Status: 3S OK (UNLOCKED)"));
      }

      footer(PSTR("[ Press knob to return ]"));

    } else if (screen == RESET_PRESETS) {
      oled.setFont(u8g2_font_6x10_tf);
      centered(PSTR("Remove added presets?"), 26);
      oled.setFont(u8g2_font_5x7_tf);
      centered(PSTR("Restore 8,10,20,50,100 mL"), 39);
      centered(PSTR("Brightness stays unchanged"), 49);
      footer(PSTR("Trigger:Reset Knob:Cancel"));

    } else if (screen == BRIGHTNESS) {
      oled.setFont(u8g2_font_helvB24_tn);
      utoa(editBrightness, text, 10);
      centeredRAM(text, 39);
      oled.setFont(u8g2_font_5x7_tf);
      centered(PSTR("percent"), 50);
      footer(PSTR("Turn:Adjust Press:Save"));

    } else if (screen == ABOUT) {
      oled.setFont(u8g2_font_6x10_tf);
      centered(PSTR("AetherFlow"), 22);
      oled.setFont(u8g2_font_5x7_tf);
      centered(PSTR("Precision Perfume Filler"), 35);
      oled.drawHLine(24, 40, 80);
      centered(PSTR("Yatharth | Pratham"), 48);
      centered(PSTR("Sahil | Ombeer"), 55);
      footer(PSTR("[ Press knob to return ]"));

    } else if (screen == MENU) {
      uint8_t menuCount = PRESET_COUNT + 5;
      int first = constrain((int)selected - 1, 0, (int)menuCount - 3);
      oled.setFont(u8g2_font_6x10_tf);

      for (uint8_t row = 0; row < 3; row++) {
        uint8_t index = first + row;
        uint8_t y = 14 + row * 13;

        if (index == selected) {
          oled.drawRBox(2, y, 116, 12, 2);
          oled.setDrawColor(0);
        }

        if (index < PRESET_COUNT) {
          utoa(PRESETS[index], text, 10);
          strcat_P(text, PSTR(" mL"));
          oled.drawStr(9, y + 9, text);
        } else {
          const char *label =
            index == PRESET_COUNT ? PSTR("Custom volume...") :
            index == PRESET_COUNT + 1 ? PSTR("Manual dispense...") :
            index == PRESET_COUNT + 2 ? PSTR("Flush mode...") :
            index == PRESET_COUNT + 3 ? PSTR("Settings...") :
            PSTR("Flow calibration...");
          drawFlashStr(9, y + 9, label);
        }

        if (index == selected) drawFlashStr(107, y + 9, PSTR(">"));
        oled.setDrawColor(1);
      }

      oled.drawVLine(124, 15, 35);
      oled.drawBox(122, 15 + (uint16_t)selected * 30 / (menuCount - 1), 5, 5);
      footer(pgm_read_byte(notice) ? notice : PSTR("Turn:Browse  Press:Set"));

    } else if (screen == CALIBRATION) {
      oled.setFont(u8g2_font_helvB24_tn);
      centeredRAM("10", 38);
      oled.setFont(u8g2_font_5x7_tf);
      centered(lowBatteryLocked ? strLowBatt
                                : PSTR("10 sec fixed timed dispense"), 49);
      footer(PSTR("Trigger:Start  Knob:Back"));

    } else if (screen == CALIBRATING) {
      unsigned long elapsed = millis() - fillStartedAt;
      if (elapsed > CAL_FIXED_DURATION_MS) elapsed = CAL_FIXED_DURATION_MS;
      uint16_t remSec = (uint16_t)((CAL_FIXED_DURATION_MS - elapsed + 999UL) / 1000UL);

      oled.setFont(u8g2_font_6x10_tf);
      centered(PSTR("Dispensing for 10s"), 25);

      oled.setFont(u8g2_font_helvB24_tn);
      utoa(remSec, text, 10);
      centeredRAM(text, 49);
      oled.setFont(u8g2_font_5x7_tf);
      centered(PSTR("Running 10s... Knob:Stop"), 59);

    } else if (screen == CAL_RESULT) {
      oled.setFont(u8g2_font_5x7_tf);
      centered(PSTR("10s done - enter mL:"), 21);

      drawVolumeTenths(calibrationTenthsML, 46, calStepTier);
      oled.drawTriangle(4, 34, 9, 30, 9, 38);
      oled.drawTriangle(123, 34, 118, 30, 118, 38);
      footer(
        calStepTier == 0 ? PSTR("Turn:0.1mL  Knob:Save") :
        calStepTier == 1 ? PSTR("Turn:1mL    Knob:Save") :
        calStepTier == 2 ? PSTR("Turn:10mL   Knob:Save") :
                           PSTR("Turn:100mL  Knob:Save")
      );

    } else if (screen == CUSTOM || screen == READY ||
               screen == ADD_PRESET) {
      drawVolume(screen == READY ? targetML : editML, 40);

      if (screen != READY) {
        oled.drawTriangle(4, 30, 9, 26, 9, 34);
        oled.drawTriangle(123, 30, 118, 26, 118, 34);
      }

      oled.setFont(u8g2_font_5x7_tf);
      centered(
        pgm_read_byte(notice)
          ? notice
          : lowBatteryLocked
            ? strLowBatt
            : screen == ADD_PRESET
              ? PSTR("Trigger: Save preset")
              : screen == CUSTOM
                ? PSTR("Trigger: Set + fill")
                : PSTR("Press trigger to fill"),
        51
      );

      footer(screen == READY
               ? PSTR("Press knob:Change volume")
               : PSTR("Turn:Adjust  Knob:Back"));

    } else if (screen == FILLING || screen == PAUSED) {
      drawVolume((uint16_t)(deliveredUL / 1000UL), 38);
      oled.setFont(u8g2_font_5x7_tf);
      strcpy_P(text, PSTR("of "));
      utoa(targetML, text + 3, 10);
      strcat_P(text, PSTR(" mL"));
      centeredRAM(text, 48);
      oled.drawRFrame(7, 51, 114, 5, 2);

      uint8_t width = (uint8_t)min(
        112UL,
        (unsigned long)deliveredUL * 112UL /
        ((uint32_t)targetML * 1000UL)
      );
      if (width) oled.drawBox(8, 52, width, 3);

      footer(screen == PAUSED
               ? (lowBatteryLocked ? PSTR("LOW BATT! Hold:Reset")
                                   : PSTR("Tap:Resume Hold:Reset"))
               : PSTR("Tap:Pause Hold:Reset"));

    } else if (screen == MANUAL_DISPENSE || screen == FLUSH_MODE) {
      uint32_t shownML = deliveredUL / 1000UL;
      if (shownML > 65535UL) shownML = 65535UL;
      drawVolume((uint16_t)shownML, 38);
      oled.setFont(u8g2_font_5x7_tf);

      centered(lowBatteryLocked
                 ? strLowBatt
                 : screen == FLUSH_MODE
                   ? (flushRunning
                        ? PSTR("Flushing for 1.2 sec")
                        : (flushStartedAt ? PSTR("Flush complete")
                                          : PSTR("Press trigger to flush")))
                   : PSTR("Manual volume estimate"),
               49);

      footer(screen == FLUSH_MODE
               ? (flushRunning ? PSTR("Press knob: Back")
                               : PSTR("Trigger:Flush  Knob:Back"))
               : (pumpRunningState
                    ? PSTR("PUMP ACTIVE - release to stop")
                    : PSTR("Hold trigger to run  Knob:Back")));

    } else {
      if (screen == COMPLETE) {
        oled.drawLine(54, 20, 61, 27);
        oled.drawLine(61, 27, 74, 15);
        oled.drawLine(54, 21, 61, 28);
        oled.drawLine(61, 28, 74, 16);
      } else {
        oled.drawBox(57, 18, 5, 10);
        oled.drawBox(66, 18, 5, 10);
      }

      oled.setFont(u8g2_font_6x10_tf);
      utoa((uint16_t)(deliveredUL / 1000UL), text, 10);
      uint8_t l = strlen(text);
      text[l] = '.';
      text[l + 1] = (char)('0' + ((uint16_t)(deliveredUL % 1000UL) / 100U));
      strcpy_P(text + l + 2, PSTR(" / "));
      utoa(targetML, text + l + 5, 10);
      strcat_P(text, PSTR(" mL"));
      centeredRAM(text, 44);
      footer(pgm_read_byte(notice) ? notice : PSTR("Release buttons to reset"));
    }
  } while (oled.nextPage() && !Wire.getWireTimeoutFlag());

  servicePumpSafetyInline();
  recoverI2CAndOLEDIfGlitched();
}

void drawBoot(unsigned long elapsed) {
  uint8_t visible = min(
    (unsigned long)BOOT_LOGO_W,
    elapsed * BOOT_LOGO_W / 640UL
  );

  oled.firstPage();
  do {
    wdt_reset();
    updateBootBuzzer(millis() - bootStartedAt);
    oled.drawXBMP(4, 18, BOOT_LOGO_W, BOOT_LOGO_H, bootLogo);

    if (visible < BOOT_LOGO_W) {
      oled.setDrawColor(0);
      oled.drawBox(4 + visible, 18, BOOT_LOGO_W - visible, BOOT_LOGO_H);
      oled.setDrawColor(1);
    }

    if (elapsed >= 640) {
      uint8_t length = min(72UL, (elapsed - 640) * 72UL / 720UL);
      if (length) oled.drawHLine(28, 54, length);
    }
  } while (oled.nextPage() && !Wire.getWireTimeoutFlag());
  updateBootBuzzer(millis() - bootStartedAt);
  recoverI2CAndOLEDIfGlitched();
}

void setup() {
  digitalWrite(PUMP_PIN, LOW);
  pinMode(PUMP_PIN, OUTPUT);
  pumpOff();

  // Initialize passive buzzer on D10 (connected between D10 and GND)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BATTERY_PIN, INPUT);
  digitalWrite(BATTERY_PIN, LOW); // Ensure internal pull-up on A0 is OFF
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  knob.begin(ENC_SW, 15);
  trigger.begin(TRIGGER, 25);

  loadSettings();
  loadTimeCalibration();
  targetML = PRESETS[0];

  // Take initial battery reading before boot animation starts
  updateBattery(millis(), true);

  encoderPrevious = (PIND >> 3) & 3;
  PCIFR |= _BV(PCIF2);
  PCMSK2 |= _BV(PCINT19) | _BV(PCINT20);
  PCICR |= _BV(PCIE2);

  oled.setI2CAddress(OLED_ADDRESS << 1);
  oled.begin();
  configureI2CSafety();
  oled.setContrast((uint16_t)brightness * 255 / 100);

  // Enable 2-Second Hardware Watchdog Timer
  wdt_enable(WDTO_2S);

  bootStartedAt = millis();
}

void loop() {
  wdt_reset();
  unsigned long now = millis();

  knob.update(now);
  trigger.update(now);
  int16_t movement = readEncoder();
  updateBattery(now);

  if (bootActive) {
    unsigned long elapsed = now - bootStartedAt;
    updateBootBuzzer(elapsed);

    if (elapsed >= BOOT_DURATION_MS) {
      stopBootBuzzer();
      bootAudioDone = true;
      bootActive = false;
      knob.begin(ENC_SW, 15);
      trigger.begin(TRIGGER, 25);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        encoderTicks = 0;
      }
      drawScreen();
      lastFrame = millis();

    } else if (now - lastFrame >= 60) {
      drawBoot(elapsed);
      lastFrame = millis();
    }
    return;
  }

  if (pgm_read_byte(notice) && (long)(now - noticeUntil) >= 0) notice = emptyNotice;

  bool triggerTap = trigger.clicked && !ignoreStartTrigger;
  bool triggerReset = trigger.held && !ignoreStartTrigger;

  // Require at least 300ms after fill start before clearing ignoreStartTrigger so trigger release bounce cannot pause fill
  if (!trigger.down && !trigger.rawDown && (screen != FILLING || (now - fillStartedAt >= 300UL)))
    ignoreStartTrigger = false;

  if (screen == FILLING || screen == PAUSED) {
    // Always check fill completion FIRST before processing any pause/button logic
    if (screen == FILLING) {
      unsigned long liveNow = millis();
      if (accumulatedFillMs + (liveNow - fillStartedAt) >= targetDurationMs) {
        deliveredUL = (uint32_t)targetML * 1000UL;
        finishFill(COMPLETE, liveNow);
      }
    }

    if (screen == FILLING || screen == PAUSED) {
      if (triggerReset) {
        pumpOff();
        deliveredUL = 0;
        accumulatedFillMs = 0;
        screen = READY;

      } else if (knob.pressed) {
        if (screen == FILLING) {
          accumulatedFillMs += (millis() - fillStartedAt);
        }
        deliveredUL = msToDeliveredUL(accumulatedFillMs);
        finishFill(STOPPED, millis());

      } else if (triggerTap) {
        if (screen == FILLING) {
          pumpOff();
          accumulatedFillMs += (millis() - fillStartedAt);
          deliveredUL = msToDeliveredUL(accumulatedFillMs);
          screen = PAUSED;
        } else {
          if (!checkBatteryLockout(now)) {
            screen = FILLING;
            fillStartedAt = millis();
            pumpOn();
          }
        }
      }
    }
  }

  switch (screen) {
    case MENU:
      selected = constrain((int)selected + movement,
                           0, (int)PRESET_COUNT + 4);

      if (knob.pressed) {
        if (selected < PRESET_COUNT) {
          targetML = PRESETS[selected];
          screen = READY;

        } else if (selected == PRESET_COUNT) {
          editML = customML;
          lastTurn = 0;
          screen = CUSTOM;

        } else if (selected == PRESET_COUNT + 1) {
          accumulatedFillMs = 0;
          deliveredUL = 0;
          pumpOff();
          screen = MANUAL_DISPENSE;

        } else if (selected == PRESET_COUNT + 2) {
          accumulatedFillMs = 0;
          deliveredUL = 0;
          flushRunning = false;
          flushStartedAt = 0;
          pumpOff();
          screen = FLUSH_MODE;

        } else if (selected == PRESET_COUNT + 3) {
          settingsSelected = 0;
          screen = SETTINGS;

        } else {
          lastTurn = 0;
          screen = CALIBRATION;
        }
      }
      break;

    case CUSTOM:
    case ADD_PRESET:
      if (movement) {
        int step = (lastTurn != 0 && now - lastTurn < 70) ? 5 : 1;
        editML = constrain((long)editML + (long)movement * step,
                           (long)MIN_ML, (long)MAX_ML);
        lastTurn = now;
      }

      if (knob.pressed) {
        screen = screen == ADD_PRESET ? SETTINGS : MENU;

      } else if (trigger.pressed) {
        if (screen == ADD_PRESET) {
          bool duplicate = false;
          for (uint8_t i = 0; i < PRESET_COUNT; i++)
            if (PRESETS[i] == editML) duplicate = true;

          if (duplicate) {
            notice = PSTR("Already in presets");
            noticeUntil = now + 1800;

          } else if (PRESET_COUNT < MAX_PRESETS) {
            uint8_t pos = PRESET_COUNT;
            while (pos && PRESETS[pos - 1] > editML) {
              PRESETS[pos] = PRESETS[pos - 1];
              pos--;
            }

            PRESETS[pos] = editML;
            PRESET_COUNT++;
            selected = PRESET_COUNT + 3;
            saveSettings();
            notice = PSTR("Preset saved");
            noticeUntil = now + 1800;
            screen = SETTINGS;
          }

        } else {
          customML = editML;
          targetML = editML;
          startFill(now);
        }
      }
      break;

    case CALIBRATION:
      if (knob.pressed) {
        screen = MENU;
      } else if (trigger.pressed) {
        startManualCalibration(now);
      }
      break;

    case CALIBRATING:
      if (now - fillStartedAt >= CAL_FIXED_DURATION_MS) {
        finishManualCalibration(now);
      } else if (knob.pressed) {
        pumpOff();
        screen = CALIBRATION;
      }
      break;

    case CAL_RESULT:
      // 1. Trigger Press (short tap or 1s hold): cycles step size (0.1 -> 1 -> 10 -> 100 -> 0.1 mL)
      //    and immediately flickers the corresponding digit place (.1, 1, 10, or 100)
      if (trigger.clicked || trigger.held) {
        calStepTier = (calStepTier + 1) % 4;
        lastTurn = 0;
      }

      // 2. Black Knob Turn: adjusts the flickering digit place (0.1, 1, 10, or 100 mL), clamped 0.1 to 500.0 mL
      if (movement) {
        long stepTenths = (calStepTier == 0) ? 1L :
                          (calStepTier == 1) ? 10L :
                          (calStepTier == 2) ? 100L : 1000L;
        calibrationTenthsML = (uint16_t)constrain(
          (long)calibrationTenthsML + (long)movement * stepTenths,
          1L, (long)MAX_ML * 10L
        );
        lastTurn = now;
      }

      // 3. Black Knob Press: saves calibratedTenthsML (volume per 10.0s) to EEPROM and returns to MENU
      if (knob.pressed) {
        calibratedTenthsML = calibrationTenthsML;
        saveTimeCalibration();
        screen = MENU;
      }
      break;

    case SETTINGS:
      settingsSelected = constrain((int)settingsSelected + movement, 0, 5);

      if (knob.pressed) {
        notice = emptyNotice;

        if (settingsSelected == 0) {
          updateBattery(now, true);
          screen = BATTERY_INFO;

        } else if (settingsSelected == 1) {
          editBrightness = brightness;
          screen = BRIGHTNESS;

        } else if (settingsSelected == 2) {
          if (PRESET_COUNT >= MAX_PRESETS) {
            notice = PSTR("Preset limit: 8");
            noticeUntil = now + 1800;
          } else {
            editML = customML;
            lastTurn = 0;
            screen = ADD_PRESET;
          }

        } else if (settingsSelected == 3) {
          screen = RESET_PRESETS;

        } else if (settingsSelected == 4) {
          screen = ABOUT;

        } else {
          selected = PRESET_COUNT + 3;
          screen = MENU;
        }
      }
      break;

    case BATTERY_INFO:
      if (knob.pressed) screen = SETTINGS;
      break;

    case RESET_PRESETS:
      if (knob.pressed) {
        screen = SETTINGS;

      } else if (trigger.pressed) {
        const uint16_t defaults[] = {8, 10, 20, 50, 100};
        PRESET_COUNT = 5;

        for (uint8_t i = 0; i < MAX_PRESETS; i++)
          PRESETS[i] = i < 5 ? defaults[i] : 0;

        selected = PRESET_COUNT + 3;
        saveSettings();
        notice = PSTR("Default presets restored");
        noticeUntil = now + 1800;
        screen = SETTINGS;
      }
      break;

    case BRIGHTNESS:
      if (movement) {
        editBrightness = constrain((int)editBrightness + movement * 5,
                                   10, 100);
        oled.setContrast((uint16_t)editBrightness * 255 / 100);
        recoverI2CAndOLEDIfGlitched();
      }

      if (knob.pressed) {
        if (brightness != editBrightness) {
          brightness = editBrightness;
          saveSettings();
        }
        screen = SETTINGS;
      }
      break;

    case ABOUT:
      if (knob.pressed) screen = SETTINGS;
      break;

    case READY:
      if (knob.pressed) {
        screen = MENU;
      } else if (trigger.pressed) {
        startFill(now);
      }
      break;

    case MANUAL_DISPENSE: {
      if (trigger.down && !lowBatteryLocked) {
        if (!pumpRunningState) {
          pumpOn();
          fillStartedAt = millis();
        }
        deliveredUL = msToDeliveredUL(accumulatedFillMs + (millis() - fillStartedAt));
      } else {
        if (trigger.pressed && lowBatteryLocked) {
          checkBatteryLockout(now);
        }
        if (pumpRunningState) {
          accumulatedFillMs += (now - fillStartedAt);
          pumpOff();
        }
        deliveredUL = msToDeliveredUL(accumulatedFillMs);
      }

      if (knob.pressed) {
        pumpOff();
        screen = MENU;
      }
      break;
    }

    case FLUSH_MODE: {
      if (!flushRunning && trigger.pressed) {
        if (!checkBatteryLockout(now)) {
          deliveredUL = 0;
          flushRunning = true;
          pumpOn();
          flushStartedAt = millis();
        }
      } else if (flushRunning && now - flushStartedAt >= FLUSH_DURATION_MS) {
        pumpOff();
        flushRunning = false;
        deliveredUL = msToDeliveredUL(FLUSH_DURATION_MS);
      }

      if (flushRunning) {
        deliveredUL = msToDeliveredUL(now - flushStartedAt);
      }

      if (knob.pressed) {
        pumpOff();
        flushRunning = false;
        screen = MENU;
      }
      break;
    }

    case FILLING: {
      unsigned long liveNow = millis();
      unsigned long elapsedMs = accumulatedFillMs + (liveNow - fillStartedAt);
      if (elapsedMs >= targetDurationMs) {
        deliveredUL = (uint32_t)targetML * 1000UL;
        finishFill(COMPLETE, liveNow);
      } else {
        deliveredUL = msToDeliveredUL(elapsedMs);
      }
      break;
    }

    // COMPLETE ("DONE") screen reduced from 1500 ms to 350 ms for fast bottle filling
    case COMPLETE:
      if (now - resultAt >= COMPLETE_HOLD_MS &&
          !trigger.down && !trigger.rawDown && !knob.down)
        screen = READY;
      break;

    case STOPPED:
      if ((now - resultAt >= STOPPED_HOLD_MS || knob.pressed || trigger.pressed) &&
          !trigger.down && !trigger.rawDown && !knob.down)
        screen = READY;
      break;

    case PAUSED:
      break;
  }

  // Do not start a new multi-page I2C OLED redraw during the final 180ms before targetDurationMs cutoff
  // so loop() can shut off D9 with sub-millisecond precision.
  bool nearCutoff = (screen == FILLING) &&
                    ((accumulatedFillMs + (millis() - fillStartedAt) + 180UL) >= targetDurationMs);

  unsigned long frameInterval = (screen == CALIBRATING)
                                  ? 1000UL
                                  : (pumpRunningState ? 250UL : 70UL);
  if (!nearCutoff && (millis() - lastFrame >= frameInterval)) {
    drawScreen();
    lastFrame = millis();
  }
}
