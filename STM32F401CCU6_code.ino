/*-----------------------------------------------------------
   STM32F401CCU6 (Black Pill)
   NANO-Chip Aero-Engine RT Sensor Fusion + Anomaly Detection
-------------------------------------------------------------
   Pin Assignments:
   SW-420      -> PA1
   MPU6050 SDA -> PB9
   MPU6050 SCL -> PB8
   UART TX     -> PA9  (to ESP32 GPIO16)
   UART RX     -> PA10 (from ESP32 GPIO17)
   Buzzer      -> PB5
   LED         -> PC13
-------------------------------------------------------------
   UART Packet Format (every 100 ms):
   START|ax|ay|az|gx|gy|gz|shock|END
-----------------------------------------------------------*/

#include <Wire.h>
#include <math.h>

// ===== Hardware Serial for USART1 (PA9/PA10) =====
HardwareSerial SerialSTM32(USART1);

// ===== PIN DEFINITIONS =====
#define SW420_PIN  PA1
#define LED_PIN    PC13
#define BUZZER_PIN PB5

// MPU6050 address & registers
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// MPU scales
#define ACCEL_SENS 16384.0f   // ±2g
#define GYRO_SENS 131.0f      // ±250°/s

// Debounce
#define DEBOUNCE_MS 50

// Thresholds
#define VIB_THRESHOLD 2.0f
#define GYRO_TILT_THRESHOLD 80.0f

// Sensor variables
float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;
float vibration_mag = 0;
int shock_state = 0;

// Debounce
int lastRawShock = 0;
int stableShock = 0;
unsigned long lastShockChange = 0;

// Packet timer
unsigned long lastSend = 0;

// ====== Setup ======
void setup() {
  pinMode(SW420_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // UART (PA9/PA10)
  SerialSTM32.begin(115200);

  // I2C on PB9/PB8
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  delay(100);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();

  delay(200);
}

// ====== MAIN LOOP ======
void loop() {
  readMPU();
  readShock();
  computeVibration();

  bool alarm = evaluateAlarm();

  digitalWrite(BUZZER_PIN, alarm ? HIGH : LOW);
  digitalWrite(LED_PIN, alarm ? HIGH : LOW);

  if (millis() - lastSend >= 100) {
    lastSend = millis();
    sendSerialPacket();
  }

  delay(5);
}

// ====== MPU6050 ======
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() < 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  int16_t temp = (Wire.read() << 8) | Wire.read(); // ignore

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  ax_g = ax / ACCEL_SENS;
  ay_g = ay / ACCEL_SENS;
  az_g = az / ACCEL_SENS;

  gx_dps = gx / GYRO_SENS;
  gy_dps = gy / GYRO_SENS;
  gz_dps = gz / GYRO_SENS;
}

// ====== SW-420 (debounced) ======
void readShock() {
  int raw = digitalRead(SW420_PIN);

  if (raw != lastRawShock) {
    lastShockChange = millis();
    lastRawShock = raw;
  }

  if (millis() - lastShockChange > DEBOUNCE_MS) {
    stableShock = raw;
  }

  shock_state = stableShock ? 1 : 0;
}

// ====== Vibration magnitude ======
void computeVibration() {
  vibration_mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
}

// ====== Alarm evaluation ======
bool evaluateAlarm() {
  if (shock_state == 1) return true;
  if (vibration_mag > VIB_THRESHOLD) return true;
  if (fabs(gx_dps) > GYRO_TILT_THRESHOLD) return true;
  if (fabs(gy_dps) > GYRO_TILT_THRESHOLD) return true;

  return false;
}

// ====== UART Packet ======
void sendSerialPacket() {
  char axb[12], ayb[12], azb[12], gxb[12], gyb[12], gzb[12];

  dtostrf(ax_g, 0, 2, axb);
  dtostrf(ay_g, 0, 2, ayb);
  dtostrf(az_g, 0, 2, azb);
  dtostrf(gx_dps, 0, 2, gxb);
  dtostrf(gy_dps, 0, 2, gyb);
  dtostrf(gz_dps, 0, 2, gzb);

  char packet[200];
  snprintf(packet, sizeof(packet),
           "START|%s|%s|%s|%s|%s|%s|%d|END\n",
           axb, ayb, azb, gxb, gyb, gzb, shock_state);

  SerialSTM32.print(packet);
}
