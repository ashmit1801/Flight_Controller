/*
  ╔══════════════════════════════════════════════════════════════╗
  ║   FIXED WING FC — Aileron Control + Full Telemetry          ║
  ║   Board  : Arduino Nano 33 BLE Rev2 (ABX00072)              ║
  ║   IMU    : BMI270 + BMM150 (built-in)                       ║
  ║   Baro   : BMP180 (external I2C)                            ║
  ║   Servos : SG90  → D6  (left elevon)                        ║
  ║            MG90S → D7  (right elevon)                       ║
  ║   Output : JSON over USB Serial @ 115200                    ║
  ╚══════════════════════════════════════════════════════════════╝

  BMP180 wiring → Nano 33 BLE Rev2:
    VCC → 3.3V    GND → GND
    SDA → A4      SCL → A5

  Servo wiring:
    SG90  signal → D6,  MG90S signal → D7
    ⚠ Power servos from an external 5V BEC — NOT from the Arduino pin.

  Libraries (Tools → Manage Libraries):
    • Arduino_BMI270_BMM150   (bundled with board package)
    • Adafruit BMP085 Library
    • Adafruit Unified Sensor

  Anti-jitter measures applied:
    1. Complementary filter (98/2) replaces raw accel attitude
    2. EMA servo smoother on top of CF output
    3. Dead-band: ignores corrections < 1° roll to prevent buzzing
    4. Integer rounding before write() to stop micro-stepping noise
    5. Baro throttled to 100 ms so it never blocks the servo loop
    6. dt clamped — stalls from Serial.print never corrupt the filter
    7. Serial output rate decoupled from servo rate (servo=50Hz, telem=20Hz)
*/

#include <Arduino_BMI270_BMM150.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// ════════════════════════════════════════
//  TUNING CONSTANTS  ← adjust for your airframe
// ════════════════════════════════════════
const int   SG90_CENTER  = 90;    // left elevon neutral trim  (0–180)
const int   MG90_CENTER  = 90;    // right elevon neutral trim (0–180)
const float AILERON_GAIN = 2.0;   // servo°/roll° — lower = less aggressive
const int   MAX_THROW    = 65;    // max deflection from centre (degrees)
const int   DEAD_BAND    = 1;     // roll error < this → no correction (stops buzz)

// Complementary filter: 0.98 = smooth, 0.95 = more responsive
const float CF_ALPHA     = 0.98;

// EMA servo smoother: 0.15 = very smooth, 0.30 = faster response
// If servos feel sluggish increase this; if they jitter decrease it
const float SERVO_EMA    = 0.20;

// Sea-level pressure (Pa) — tune for your local QNH
float seaLevelPa = 101325.0;

// ════════════════════════════════════════
//  OBJECTS
// ════════════════════════════════════════
Adafruit_BMP085 bmp;
Servo sg90;   // left elevon
Servo mg90;   // right elevon

// ════════════════════════════════════════
//  SENSOR READINGS
// ════════════════════════════════════════
float accelX = 0, accelY = 0, accelZ = 1;
float gyroX  = 0, gyroY  = 0, gyroZ  = 0;
float magX   = 0, magY   = 0, magZ   = 0;

// ════════════════════════════════════════
//  ATTITUDE
// ════════════════════════════════════════
float roll    = 0;
float pitch   = 0;
float yaw     = 0;
float roll_cf  = 0;   // complementary filter accumulators
float pitch_cf = 0;

// ════════════════════════════════════════
//  BARO
// ════════════════════════════════════════
float altitude    = 0;
float pressure    = 101325;
float temperature = 25;

// ════════════════════════════════════════
//  SERVO STATE
// ════════════════════════════════════════
float sg90Smooth = SG90_CENTER;   // EMA accumulators (float for precision)
float mg90Smooth = MG90_CENTER;
int   sg90Pos    = SG90_CENTER;   // actual written values
int   mg90Pos    = MG90_CENTER;

// ════════════════════════════════════════
//  TIMING
// ════════════════════════════════════════
unsigned long lastUs      = 0;    // micros — for dt
unsigned long lastBaroMs  = 0;    // millis — baro throttle
unsigned long lastTelemMs = 0;    // millis — serial output throttle

const unsigned long BARO_MS  = 100;   // baro every 100 ms
const unsigned long TELEM_MS = 50;    // telemetry every 50 ms (20 Hz)
//                                       servo write happens EVERY loop (~50 Hz)

// ════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Don't freeze if no PC is connected —
  // wait max 2 s then continue standalone
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000);

  Serial.println("================================");
  Serial.println(" FIXED WING FC INITIALIZING    ");
  Serial.println("================================");

  // ── Servos to neutral first ──
  sg90.attach(6);
  mg90.attach(7);
  sg90.write(SG90_CENTER);
  mg90.write(MG90_CENTER);
  delay(500);   // let servos reach neutral before IMU reads
  Serial.println("SERVOS ATTACHED — NEUTRAL");

  // ── IMU ──
  if (!IMU.begin()) {
    Serial.println("ERROR: IMU FAILED — HALTING");
    while (1);
  }
  Serial.println("IMU OK");

  // ── BMP180 ──
  if (!bmp.begin()) {
    Serial.println("ERROR: BMP180 NOT FOUND");
    // Remove the while(1) below to run without baro (altitude = 0)
    while (1);
  }
  Serial.println("BMP180 OK");

  // ── Seed CF filter with first accel reading ──
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    roll_cf  = atan2(accelY, accelZ) * 180.0 / PI;
    pitch_cf = atan2(-accelX,
                     sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  }
  sg90Smooth = SG90_CENTER;
  mg90Smooth = MG90_CENTER;

  lastUs = micros();
  Serial.println("SYSTEM READY");
  Serial.println("AILERON CONTROL ACTIVE");
}

// ════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════
void loop() {

  // ── dt (seconds) ────────────────────────────────
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) / 1000000.0f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.02f;  // clamp: ignore stall spikes

  // ── Read IMU ─────────────────────────────────────
  if (IMU.accelerationAvailable())
    IMU.readAcceleration(accelX, accelY, accelZ);

  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

  if (IMU.magneticFieldAvailable())
    IMU.readMagneticField(magX, magY, magZ);

  // ── Complementary filter: roll & pitch ──────────
  float ar = atan2(accelY, accelZ) * 180.0f / PI;
  float ap = atan2(-accelX,
                   sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;

  roll_cf  = CF_ALPHA * (roll_cf  + gyroX * dt) + (1.0f - CF_ALPHA) * ar;
  pitch_cf = CF_ALPHA * (pitch_cf + gyroY * dt) + (1.0f - CF_ALPHA) * ap;

  roll  = roll_cf;
  pitch = pitch_cf;

  // ── Tilt-compensated yaw ─────────────────────────
  float rr = roll  * PI / 180.0f;
  float pr = pitch * PI / 180.0f;
  float mxc =  magX * cos(pr) + magZ * sin(pr);
  float myc =  magX * sin(rr) * sin(pr)
             + magY * cos(rr)
             - magZ * sin(rr) * cos(pr);
  yaw = atan2(-myc, mxc) * 180.0f / PI;
  if (yaw < 0) yaw += 360.0f;

  // ── BMP180 — throttled, never blocks servo loop ──
  unsigned long nowMs = millis();
  if (nowMs - lastBaroMs >= BARO_MS) {
    pressure    = (float)bmp.readPressure();
    temperature = bmp.readTemperature();
    altitude    = bmp.readAltitude(seaLevelPa);
    lastBaroMs  = nowMs;
  }

  // ════════════════════════════════════════
  //  AILERON CONTROL — jitter-free
  // ════════════════════════════════════════

  // 1. Dead-band: small rolls ignored → stops servo buzzing at rest
  float rollFiltered = (fabs(roll) < DEAD_BAND) ? 0.0f : roll;

  // 2. Compute target position
  float correction = constrain(rollFiltered * AILERON_GAIN,
                                -(float)MAX_THROW, (float)MAX_THROW);
  float sg90Target = SG90_CENTER + correction;
  float mg90Target = MG90_CENTER - correction;

  // 3. EMA smooth — prevents step-changes that cause jitter
  sg90Smooth = SERVO_EMA * sg90Target + (1.0f - SERVO_EMA) * sg90Smooth;
  mg90Smooth = SERVO_EMA * mg90Target + (1.0f - SERVO_EMA) * mg90Smooth;

  // 4. Round to nearest integer — write() only accepts int;
  //    fractional values cause micro-stepping noise
  int sg90New = (int)(sg90Smooth + 0.5f);
  int mg90New = (int)(mg90Smooth + 0.5f);

  sg90New = constrain(sg90New, 10, 170);
  mg90New = constrain(mg90New, 10, 170);

  // 5. Only call write() if value actually changed — avoids
  //    redundant PWM pulses that make servos twitch
  if (sg90New != sg90Pos) { sg90.write(sg90New); sg90Pos = sg90New; }
  if (mg90New != mg90Pos) { mg90.write(mg90New); mg90Pos = mg90New; }

  // ════════════════════════════════════════
  //  JSON TELEMETRY — rate limited to 20 Hz
  //  (servo loop runs at ~50 Hz independently)
  // ════════════════════════════════════════
  nowMs = millis();
  if (nowMs - lastTelemMs >= TELEM_MS) {
    lastTelemMs = nowMs;

    // Single Serial.print block — no intermediate flushes
    // so the UART buffer fills and sends in one burst
    char buf[320];
    snprintf(buf, sizeof(buf),
      "{"
      "\"roll\":%.2f,"
      "\"pitch\":%.2f,"
      "\"yaw\":%.2f,"
      "\"altitude\":%.2f,"
      "\"pressure\":%.0f,"
      "\"temperature\":%.2f,"
      "\"accelX\":%.3f,"
      "\"accelY\":%.3f,"
      "\"accelZ\":%.3f,"
      "\"gyroX\":%.3f,"
      "\"gyroY\":%.3f,"
      "\"gyroZ\":%.3f,"
      "\"magX\":%.3f,"
      "\"magY\":%.3f,"
      "\"magZ\":%.3f,"
      "\"sg90\":%d,"
      "\"mg90\":%d"
      "}",
      roll, pitch, yaw,
      altitude, pressure, temperature,
      accelX, accelY, accelZ,
      gyroX, gyroY, gyroZ,
      magX, magY, magZ,
      sg90Pos, mg90Pos
    );
    Serial.println(buf);   // println adds \n — required by dashboard parser
  }
}
