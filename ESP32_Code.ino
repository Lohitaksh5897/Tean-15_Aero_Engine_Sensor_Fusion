/**
 * ESP32 - NANO-Chip Aero-Engine Monitoring System
 * FINAL VERSION WITH DHT11 (NO LM35)
 *
 * STM32 sends ONLY: START|ax|ay|az|gx|gy|gz|shock|END
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ========== DHT11 CONFIG ==========
#define DHTPIN 26
#define DHTTYPE DHT11
DHT dht11(DHTPIN, DHTTYPE);

// ========== WIFI / AWS CONFIG ==========
const char* ssid = "samsung21";
const char* password = "Vinay1122";
const char* THINGNAME = "Nano_Aero_Engine";
const char* mqtt_server = "a10ndj6af7jzj8-ats.iot.us-east-1.amazonaws.com";
const int mqtt_port = 8883;
const char* mqtt_topic = "nanochip/aero";

const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char* client_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUfV2OrQ+I/bpNygwEcicvxBoE3RMwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MTExNTA4MTAx
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAODzJGBjPAuXwyGQVF8K
ZRg8QyxeL4ODE64BilOlF1qxDL4xhB4PC0rj3iIyZxqz4pxRKwbiruQkwav9LGIF
SbRASbx3degkHZEfylu3JQZZB1irAVHJIUTrcUh6vw26YB52/b8+2Ix28QIoqZ/g
9WH0M/+VJQcE/26KhQiW3nAiEOnQlWl3fZWi+i3QVqCqTUyv57CzFGKNsqoerZJk
VF+jzZ/SaHjLRzxe4kg3u63JnZ2T9pTsrOKt73s7/L+uzleqhk83gm2vVt0iIqN8
29BQ1ktYvzRBkeHD4sc1Ks9AS+Jsu+zBYR6ZGjcccPl5rcS7x7FtqpZ/OQ/ap3QZ
WiECAwEAAaNgMF4wHwYDVR0jBBgwFoAUW/XO1IIIcyFDd8SFXaia+g9xheQwHQYD
VR0OBBYEFBcs+xlhYgkStxDOcs0GZyxphvjPMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAbTd3V3+8/CRtnJFUStemsbrji
agD10eW8MfMDBwsiE0kUb2Y555sR0Gw0m5UMtfA4/CbUG4cY5dPVWSisYT81rpcu
rf55WHDNdcFZtG8JxZ9krhdFBjVfpxd1cVPfHZznhpCcK8FVEnIUdTWx+FXRCsIh
XdcB5hOMhNKYu25pTHDbz2V3RhpkkC6imj5+ppHtzfkoj/KU8VEEP225CpqpNYWM
Ps9RrFVPWTrvWLO9ejTNNm416P+4jPm67cixPjahkBi+40WkMhkOyWz9gJ++nPBM
y38++26uH8/69hnPKrnpfDo7T+oOpGsQcsVEqQjWjar/ycmPUVkIzmXWjIBi
-----END CERTIFICATE-----


)EOF";

const char* client_key = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEA4PMkYGM8C5fDIZBUXwplGDxDLF4vg4MTrgGKU6UXWrEMvjGE
Hg8LSuPeIjJnGrPinFErBuKu5CTBq/0sYgVJtEBJvHd16CQdkR/KW7clBlkHWKsB
UckhROtxSHq/DbpgHnb9vz7YjHbxAiipn+D1YfQz/5UlBwT/boqFCJbecCIQ6dCV
aXd9laL6LdBWoKpNTK/nsLMUYo2yqh6tkmRUX6PNn9JoeMtHPF7iSDe7rcmdnZP2
lOys4q3vezv8v67OV6qGTzeCba9W3SIio3zb0FDWS1i/NEGR4cPixzUqz0BL4my7
7MFhHpkaNxxw+XmtxLvHsW2qln85D9qndBlaIQIDAQABAoIBABEpe7T+3S3lPjXp
VQaLPHtE3bsc1HMsJOw5odwP1GBO8bHxBStl6m6kQ+ZIAiW/Y6KZbrbvxHgTddMx
VZmE3cKqxPVkB9LWLotL6ZxltKVvVWgmY8thlPA1JkdpVfouJfctDN7yZIMS8ABC
Ke7QsufuQalWL6UBdJtyZgPpvr6/NfmA/uzwu933iFUJTFxsbrQEO2pM2ETmKN1v
blASJHPu+NrW0cAP5eHagNsupN4gaxR50or359MRRstUsus4IyzCDOPTtFBL8WDo
SArARWs3osKadhebAiZ5q//1ObJ4xGgfPZ3oDmWiz6XIUyxVkLW191D9G+zPRiyV
yLwXKiECgYEA9siAmsnvo8KHjIDv2E3IludbLd3dfV4mASErAu+zL4cgL3IZAm9X
FlnSsvRoqzwCHHoIhia+63K+a0WSBYtdOUWpEM1LQCj0Pa8pBkH/ASByF5J0xQxF
n+Qq7IagzpWxfMkGUbNkSiUdx3iQ9C2EQ6ZqLIZakClygTSDgstwKtcCgYEA6Vnj
0paJcqwFZaiyi/UK7AfKuEKkTm3AsPD5nl9+iBFF7sbuaV7502gOtWKUfSde/qKt
5VSofhxSIn0qQpMwfKc9/tUKj+M7oIJEBODcdpDkLGFg3ptvXNH79VmAnV3i1Zt3
UwXfiCRtiTDTHlmkdw7SX2uQmU3Bm85wrPWru8cCgYEA2zFnmRS3tzp3bwKvBe7A
tdg0kl639jyDgQlWzLre6t4YZkvvjswaDA4oKS8RMNaSAX0ayRztAu/d1iXq9uW8
HbyT2AqgRhKaLZfA1oAT4YpSDEHXWZX22Xet8Hv7pMIs7WsYI8U01O52rs+V4Gv1
SKktMPL8yYMm40Ajdy7/J+0CgYAVFfxSMSwmYojyULYhk3jzLxkQYWU7eQtqcbv8
paBnZzgshWlRuDVW9xYtKcVFMeORmt2f1XCMRL7fQvQoo+hDu1EMz+9uZRgkPuEq
l68UP69+myYHt3/uoULlPrsyfSNcLfowVF2IjdIgDXtmSK9l2r8DxFKCp9YQ4Cyq
VGC6TQKBgQCe3tvUlKbC8hflttcS684tvtWzvhSnZ7Po7JJY0idsHbZM1F8wnTBJ
AKXH6mWfxbD3++aKizsaHiu2oQukmKZSksH09XvL1GRCnauV72VnLESUv9Rl00i2
HRmTAJtUNzls5Lb14JvvNCSJGMdP4hS4ZE2UcNmblGgzvDvcgHv1hw==
-----END RSA PRIVATE KEY-----
)EOF";

// ========== PINS ==========
#define RXD2 16
#define TXD2 17
#define LCD_ADDR 0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
WiFiClientSecure net;
PubSubClient mqttClient(net);

// ========== DATA STRUCT ==========
struct SensorData {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  int shock;
  float vibration;
  float temp;
  float humi;
  bool valid;
};

SensorData data;

unsigned long lastLCD = 0;
int lcdPage = 0;

// =====================================================================
//   DHT11 FUNCTION  (REPLACES LM35)
// =====================================================================
void dht() {
  float humi = dht11.readHumidity();
  float tempC = dht11.readTemperature();
  float tempF = dht11.readTemperature(true);

  if (isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT11 sensor!");
  } else {

    data.humi = humi;
    data.temp = tempC;

    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print(" | ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C ~ ");
    Serial.print(tempF);
    Serial.println("°F");
  }
}

// =====================================================================
//  WIFI CONNECT STATUS
// =====================================================================
void setup_wifi() {
  delay(10);
  Serial.println("\nConnecting to WiFi...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    lcd.clear();
    lcd.print("WiFi Connected");
  } else {
    Serial.println("\n✗ WiFi Connection Failed!");

    lcd.clear();
    lcd.print("WiFi Failed!");
  }
}

// =====================================================================
//  MQTT CONNECT
// =====================================================================
void reconnect_mqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to AWS IoT...");

    String clientId = "ESP32_NanoChip_" + String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" Connected!");
      lcd.clear();
      lcd.print("AWS Connected");

    } else {
      Serial.print(" Failed (rc=");
      Serial.print(mqttClient.state());
      Serial.println(")");
      Serial.println("Retrying...");

      lcd.clear();
      lcd.print("AWS Error!");
      delay(3000);
    }
  }
}

// =====================================================================
//  UART PARSE FROM STM32  (ANOMALY ADDED)
// =====================================================================
void parseUARTData(String s) {
  s.replace("START|",""); 
  s.replace("|END","");
  s.trim();

  String v[8];
  int idx=0, p=0;
  while (p < s.length() && idx < 8) {
    int n = s.indexOf('|', p);
    if (n == -1) { v[idx++] = s.substring(p); break; }
    v[idx++] = s.substring(p, n);
    p = n + 1;
  }

  if (idx < 7) return;

  data.accel_x = v[0].toFloat();
  data.accel_y = v[1].toFloat();
  data.accel_z = v[2].toFloat();
  data.gyro_x  = v[3].toFloat();
  data.gyro_y  = v[4].toFloat();
  data.gyro_z  = v[5].toFloat();
  data.shock   = v[6].toInt();

  data.vibration = sqrt(
    data.accel_x*data.accel_x +
    data.accel_y*data.accel_y +
    data.accel_z*data.accel_z
  );

  data.valid = true;

  // =================================================================
  //   ANOMALY DETECTION (ADDED AS YOU REQUESTED)
  // =================================================================
  String anomaly = "";

  if (data.vibration > 2.0) anomaly = "HighVib";
  if (abs(data.gyro_x) > 80 || abs(data.gyro_y) > 80) anomaly = "Tilt";
  if (data.shock == 1) anomaly = "Shock";

  if (anomaly != "") {

      Serial.println("⚠ ANOMALY DETECTED:  " + anomaly);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ANOMALY! "  + anomaly);   // <--- YOUR REQUESTED LINE
      delay(2000);
  }

  Serial.println("========== Data Received ==========");
  Serial.printf("Accel: %.2f %.2f %.2f\n", data.accel_x, data.accel_y, data.accel_z);
  Serial.printf("Gyro : %.2f %.2f %.2f\n", data.gyro_x, data.gyro_y, data.gyro_z);
  Serial.printf("Shock: %d\n", data.shock);
  Serial.printf("Vib  : %.2f g\n", data.vibration);
}

// =====================================================================
//  LCD DISPLAY
// =====================================================================
void displayOnLCD() {
  if (!data.valid) return;

  lcd.clear();
  switch (lcdPage) {
    case 0:
      lcd.setCursor(0,0);
      lcd.print("Temp:");
      lcd.print(data.temp,1);
      lcd.print("C");
      lcd.setCursor(0,1);
      lcd.print("Humi:");
      lcd.print(data.humi,1);
      lcd.print("%");
      break;

    case 1:
      lcd.setCursor(0,0); lcd.print("Acceleration:");
      lcd.setCursor(0,1);
      lcd.printf("%.2f %.2f %.2f", data.accel_x, data.accel_y, data.accel_z);
      break;

    case 2:
      lcd.setCursor(0,0); lcd.print("Gyroscope:");
      lcd.setCursor(0,1);
      lcd.printf("%.1f %.1f %.1f", data.gyro_x, data.gyro_y, data.gyro_z);
      break;

    case 3:
      lcd.setCursor(0,0); lcd.print("Vibration:");
      lcd.print(data.vibration,2);
      lcd.print("g");
      lcd.setCursor(0,1);
      lcd.print("Shock:");
      lcd.print(data.shock ? "YES" : "NO");
      break;
  }
}

// =====================================================================
//  AWS JSON PUBLISH
// =====================================================================
void publishToAWS() {
  StaticJsonDocument<256> doc;

  doc["temp"] = data.temp;
  doc["humi"] = data.humi;

  JsonObject accel = doc.createNestedObject("accel");
  accel["x"] = data.accel_x;
  accel["y"] = data.accel_y;
  accel["z"] = data.accel_z;

  JsonObject gyro = doc.createNestedObject("gyro");
  gyro["x"] = data.gyro_x;
  gyro["y"] = data.gyro_y;
  gyro["z"] = data.gyro_z;

  doc["shock"] = data.shock;
  doc["vibration"] = data.vibration;

  char buf[256];
  serializeJson(doc, buf);

  mqttClient.publish(mqtt_topic, buf);
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  // STARTUP MESSAGE
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("AERO ENGINE");
  lcd.setCursor(0,1);
  lcd.print("SENSOR FUSION");
  Serial.println("AERO ENGINE SENSOR FUSION");
  delay(2000);

  dht11.begin();
  setup_wifi();

  net.setCACert(ca_cert);
  net.setCertificate(client_cert);
  net.setPrivateKey(client_key);
  mqttClient.setServer(mqtt_server, mqtt_port);

  lcd.clear();
  lcd.print("Waiting STM32...");
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) reconnect_mqtt();
    mqttClient.loop();
  }

  if (Serial2.available()) {
    String s = Serial2.readStringUntil('\n');
    s.trim();
    if (s.startsWith("START")) {
      parseUARTData(s);
      dht();
      displayOnLCD();
      if (mqttClient.connected()) publishToAWS();
    }
  }

  if (millis() - lastLCD > 3000) {
    lastLCD = millis();
    lcdPage = (lcdPage + 1) % 4;
    displayOnLCD();
  }
}