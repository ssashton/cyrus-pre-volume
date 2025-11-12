/* Cyrus_Sniffer_V4_MuteAware_PREREV + PGA2310 SPI (standalone-safe)
 *
 * - Decodes 24-bit word into 2×9-bit DAC ladder values (0..511).  [UNTOUCHED]
 * - Drives a PGA2310 via hardware SPI with correct protocol.       [FIXED]
 * - Standalone safe: does not block waiting for Serial; prints only if Serial present.
 *
 * PGA2310 protocol (correct):
 *   - Send 2 bytes per update: RIGHT first, then LEFT. MSB first.
 *   - SDI latched on rising SCLK → SPI MODE0 (CPOL=0, CPHA=0).
 *   - Code N: 0=mute, 1=−95.5 dB, ..., 192=0 dB, ..., 255=+31.5 dB.
 */

#include <Arduino.h>
#include <math.h>
#include <SPI.h>

// ---------------- USER SETTINGS ----------------
// --- Cyrus digital interface (input from shift register) ---
const uint8_t PIN_DATA  = 4;   // DATA from Cyrus (serial data)
const uint8_t PIN_CLK   = 2;   // CLOCK (INT0)
const uint8_t PIN_LATCH = 3;   // LATCH (INT1)

// --- PGA2310 SPI interface (output to volume IC) ---
const uint8_t PIN_PGA_CS   = 10;  // Chip Select (active LOW)
const uint8_t PIN_PGA_SCLK = 15;  // SPI SCK (hardware SPI on Micro)
const uint8_t PIN_PGA_SDI  = 16;  // SPI MOSI (hardware SPI on Micro)
const uint8_t PIN_PGA_SDO  = 14;  // SPI MISO (from PGA) — connected but unused
const uint8_t PIN_PGA_MUTE = A0;  // Hardware MUTE pin — we force HIGH in setup and never touch again
// ZCEN is tied HIGH via 10k (always enabled)

// --- SPI setup parameters ---
const uint32_t PGA_SPI_CLOCK_HZ = 1000000UL; // 1 MHz is plenty
const uint8_t  PGA_SPI_MODE     = SPI_MODE0; // CPOL=0, CPHA=0 (PGA2310 latches on rising edge)

// --- Mapping / decode options (baseline) ---
const uint8_t  LEFT_SHIFT  = 0;     // LEFT = b8..b0
const uint8_t  RIGHT_SHIFT = 10;    // RIGHT = b18..b10
const uint16_t FIELD_MASK  = 0x1FF; // 9 bits

const bool ACTIVE_LOW = true;   // ladder active-low → invert bits
const bool REVERSE9   = true;   // fields arrive MSB→LSB → reverse
bool FLIP_POSITION    = true;   // big value = mute, small = loud → flip so 0..511 = quiet..loud

volatile int DIFF_THRESHOLD_COUNTS = 5; // ignore tiny L/R diffs (unless mute on either side)

float ALPHA     = 0.20f;    // your current feel
float ATTEN_MIN = -135.0f;  // your current bottom stop (clipped to PGA range later)
// ------------------------------------------------

// Capture state
volatile uint32_t shiftReg = 0;
volatile uint16_t edgeCount = 0;
volatile bool wordReady = false;
volatile uint32_t capturedW24 = 0;
volatile uint16_t capturedEdges = 0;

// -------- Helpers --------
static inline uint16_t bitrev9(uint16_t x) {
  uint16_t r = 0;
  for (uint8_t i = 0; i < 9; ++i) r |= ((x >> i) & 0x1) << (8 - i);
  return r & 0x1FF;
}

// Extract and apply transforms to get a natural 0..511 "position" (increasing with volume)
static inline uint16_t fieldDecoded(uint32_t w24, uint8_t start) {
  uint16_t v = (w24 >> start) & FIELD_MASK; // raw
  if (ACTIVE_LOW) v = (~v) & FIELD_MASK;    // active-low → natural
  if (REVERSE9)   v = bitrev9(v);           // MSB→LSB → LSB→MSB
  if (FLIP_POSITION) v = 511 - v;           // big=quiet → small=quiet
  return v; // 0..511 (0 = lowest volume)
}

// Continuous dB mapping (no hardware mute handling here)
float posToDB(uint16_t code) {
  float p = (float)code / 511.0f;   // 0..1 (quiet..loud)
  float shaped = powf(p, ALPHA);    // log-like feel
  float dB = ATTEN_MIN * (1.0f - shaped);
  return dB;
}

// Clip and snap to PGA2310 0.5 dB step
float snapToPGA(float dB) {
  if (dB < -95.5f) dB = -95.5f;
  if (dB > 0.0f)   dB = 0.0f;
  return round(dB / 0.5f) * 0.5f;
}

// ---- PGA2310: Convert snapped dB to device code N ----
// N mapping: 0=mute, 1=−95.5 dB, ..., 192=0 dB, ..., 255=+31.5 dB.
static inline uint8_t dB_to_pga_code(float dB_snapped) {
  if (dB_snapped > 0.0f)   dB_snapped = 0.0f;
  if (dB_snapped < -95.5f) dB_snapped = -95.5f;
  float Nf = 255.0f - 2.0f * (31.5f - dB_snapped); // 0 dB -> 192
  int   N  = (int)lroundf(Nf);
  if (N < 1)   N = 1;     // -95.5 dB
  if (N > 255) N = 255;   // +31.5 dB
  return (uint8_t)N;
}

// Drive PGA2310: two bytes per update (RIGHT then LEFT), MSB-first
static inline void pga_send(uint8_t left_code, uint8_t right_code) {
  digitalWrite(PIN_PGA_CS, LOW);
  // RIGHT first, then LEFT as per PGA2310 framing
  SPI.transfer(right_code);
  SPI.transfer(left_code);
  digitalWrite(PIN_PGA_CS, HIGH);
}

// -------- ISRs --------
void onClockEdge() {
  uint8_t bit = (uint8_t)(digitalRead(PIN_DATA) & 1);
  shiftReg = ((shiftReg << 1) | bit) & 0xFFFFFFUL;  // keep 24 LSBs
  edgeCount++;
}

void onLatchRise() {
  capturedW24   = shiftReg & 0xFFFFFFUL;
  capturedEdges = edgeCount;
  edgeCount     = 0;
  wordReady     = true;
}

// -------- Arduino --------
void setup() {
  // --- Critical pin init BEFORE any Serial wait ---
  pinMode(PIN_PGA_CS,   OUTPUT);
  pinMode(PIN_PGA_MUTE, OUTPUT);
  digitalWrite(PIN_PGA_CS, HIGH);   // idle high
  digitalWrite(PIN_PGA_MUTE, HIGH); // force unmuted (active-low pin)

  // Keep 32U4 in SPI master mode regardless of external 5V / no-USB
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);

  // Bring up Serial but DO NOT wait for it (standalone-safe)
  Serial.begin(115200);

  // Cyrus inputs and interrupts
  pinMode(PIN_DATA,  INPUT_PULLUP);
  pinMode(PIN_CLK,   INPUT_PULLUP);
  pinMode(PIN_LATCH, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_CLK),   onClockEdge, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_LATCH), onLatchRise, RISING);

  // SPI init (hardware SPI on Micro: SCK=D15, MOSI=D16, MISO=D14)
  SPI.begin();

  if (Serial) {
    Serial.println(F("Cyrus 24-bit sniffer (ATmega32U4) + PGA2310 SPI (standalone-safe)"));
    Serial.println(F("Fields: LEFT b8..b0, RIGHT b18..b10, spacer b9"));
    Serial.print (F("DIFF_THRESHOLD_COUNTS=")); Serial.println(DIFF_THRESHOLD_COUNTS);
    Serial.print (F("ACTIVE_LOW=")); Serial.print(ACTIVE_LOW);
    Serial.print (F(" REVERSE9=")); Serial.print(REVERSE9);
    Serial.print (F(" FLIP_POSITION=")); Serial.println(FLIP_POSITION);
    Serial.print (F("ALPHA=")); Serial.print(ALPHA, 2);
    Serial.print (F(" ATTEN_MIN=")); Serial.print(ATTEN_MIN, 1);
    Serial.println(F(" dB (for non-mute codes)"));
    Serial.println(F("Mute rule: DECODED field == 0 → send −95.5 dB (no HW mute pin)"));
    Serial.println(F("PGA2310: RIGHT then LEFT, MODE0, 0 dB→N=192, step=0.5 dB, −95.5 dB→N=1, N=0=mute"));
    Serial.println();
  }
}

void loop() {
  if (!wordReady) return;

  noInterrupts();
  uint32_t w24   = capturedW24;
  uint16_t edges = capturedEdges;
  wordReady = false;
  interrupts();

  // --- Decoding path (UNCHANGED) ---
  uint16_t L_pos = fieldDecoded(w24, LEFT_SHIFT);
  uint16_t R_pos = fieldDecoded(w24, RIGHT_SHIFT);

  bool L_is_mute = (L_pos == 0);
  bool R_is_mute = (R_pos == 0);

  if (!L_is_mute && !R_is_mute && abs((int)L_pos - (int)R_pos) < DIFF_THRESHOLD_COUNTS) {
    uint16_t avg = (L_pos + R_pos) / 2;
    L_pos = R_pos = avg;
  }

  float L_dB = L_is_mute ? -95.5f : posToDB(L_pos);
  float R_dB = R_is_mute ? -95.5f : posToDB(R_pos);

  float L_dB_pga = snapToPGA(L_dB);
  float R_dB_pga = snapToPGA(R_dB);

  uint8_t L_code = dB_to_pga_code(L_dB_pga);
  uint8_t R_code = dB_to_pga_code(R_dB_pga);

  // --- PGA2310 write ---
  static uint8_t prev_L_code = 255, prev_R_code = 255; // invalid to force first update
  if (L_code != prev_L_code || R_code != prev_R_code) {
    SPI.beginTransaction(SPISettings(PGA_SPI_CLOCK_HZ, MSBFIRST, PGA_SPI_MODE));
    pga_send(L_code, R_code);  // function sends RIGHT then LEFT
    SPI.endTransaction();
    prev_L_code = L_code;
    prev_R_code = R_code;
  }

  // ---- Serial printout (only if a host is connected) ----
  if (Serial) {
    char hexbuf[7]; snprintf(hexbuf, sizeof(hexbuf), "%06lX", (unsigned long)(w24 & 0xFFFFFFUL));

    Serial.print(F("w24=0x")); Serial.print(hexbuf);
    Serial.print(F("  L="));    Serial.print(L_pos);
    Serial.print(F("  R="));    Serial.print(R_pos);
    Serial.print(F("  edges="));Serial.print(edges);
    if (L_is_mute) Serial.print(F("  L_MUTE"));
    if (R_is_mute) Serial.print(F("  R_MUTE"));
    Serial.print(F("  L_dB="));      Serial.print(L_dB, 1);
    Serial.print(F("  R_dB="));      Serial.print(R_dB, 1);
    Serial.print(F("  L_dB_PGA="));  Serial.print(L_dB_pga, 1);
    Serial.print(F("  R_dB_PGA="));  Serial.print(R_dB_pga, 1);
    Serial.print(F("  L_CODE(N)=")); Serial.print(L_code);
    Serial.print(F("  R_CODE(N)=")); Serial.print(R_code);
    Serial.println();
  }
}
