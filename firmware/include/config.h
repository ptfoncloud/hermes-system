#pragma once

// ─── I²C bus ────────────────────────────────────────────────────────────────
// Teensy 4.1 default I²C: SDA=18, SCL=19
// All three sensors share one bus.

// BNO085 — default I²C address (PS1=0, PS0=0)
constexpr uint8_t BNO085_ADDR   = 0x4A;

// ADXL375 — SDO/ALT pin to GND → 0x53
constexpr uint8_t ADXL375_ADDR  = 0x53;

// BMP388 — SDO to GND → 0x76
constexpr uint8_t BMP388_ADDR   = 0x76;

// ─── SD (SPI) ───────────────────────────────────────────────────────────────
// Teensy 4.1 has onboard µSD via SDIO — use the SDIO interface, not SPI.
// BUILTIN_SDCARD is defined by Teensyduino for the onboard slot.

// ─── Power ──────────────────────────────────────────────────────────────────
// Voltage divider on A0: 100k + 100k = ÷2, so ADC reads Vbat/2.
// At 3.3V ref: Vbat = (ADC / 1023.0) * 3.3 * 2.0
constexpr uint8_t  VBAT_PIN          = A0;
constexpr float    VBAT_LOW_VOLTS    = 3.50f;   // warn below this
constexpr float    VBAT_CUTOFF_VOLTS = 3.30f;   // abort below this

// ─── Flight state machine thresholds ────────────────────────────────────────
// Tune after Phase 0 flight data is in hand.

// PAD → BOOST: sustained high-g from ADXL375 (m/s²)
constexpr float    LAUNCH_ACCEL_THRESHOLD  = 30.0f;   // ~3g
constexpr uint32_t LAUNCH_ACCEL_MIN_MS     = 50;      // must hold for 50ms

// BOOST → COAST: net accel drops near gravity (motor burnout)
constexpr float    BURNOUT_ACCEL_THRESHOLD = 15.0f;   // m/s², magnitude
constexpr uint32_t BURNOUT_ACCEL_MIN_MS    = 100;

// COAST → APOGEE: vertical velocity sign change (baro-derived)
// Positive = ascending. At apogee, dAlt/dt crosses zero.
constexpr float    APOGEE_VVEL_THRESHOLD   = 0.5f;    // m/s (near zero)

// APOGEE → DESCENT: confirmed by both sign of vvel AND altitude drop
constexpr float    DESCENT_ALT_DROP_M      = 5.0f;    // must drop 5m from peak

// DESCENT → LANDED: low accel, low vvel, stable for a while
constexpr float    LANDED_ACCEL_MAX        = 12.0f;   // m/s² (near 1g)
constexpr float    LANDED_VVEL_MAX         = 1.0f;    // m/s
constexpr uint32_t LANDED_STABLE_MS        = 2000;    // 2s stable

// ─── Logging ────────────────────────────────────────────────────────────────
constexpr uint32_t LOG_RATE_HZ        = 100;     // target sample rate
constexpr uint32_t LOG_FLUSH_EVERY_N  = 50;      // flush every N records (~0.5s)
                                                  // always flush on state change

// Binary log record size: see logger.h for struct layout
// File naming: FLIGHT_NNN.bin — NNN increments each boot if SD present

// ─── I²C clock ──────────────────────────────────────────────────────────────
constexpr uint32_t I2C_CLOCK_HZ = 400000;  // fast mode
