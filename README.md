# Iotricity-
// ESP32 Structural Health Monitoring: HX711 (strain / bridge), ADXL335 (vibration)
// Sends JSON via HTTP POST or MQTT. Tune calibration constants after lab calibration.
//
// Required libraries:
//   - HX711 (https://github.com/bogde/HX711)
//   - PubSubClient (optional, for MQTT)

#include <WiFi.h>
#include <HTTPClient.h>
#include "HX711.h"
#include <PubSubClient.h>

// ---------- CONFIG ----------
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASS";

// Choose one of send methods: useHTTP=true to POST; useMQTT=true to publish to broker
const bool useHTTP = true;
const bool useMQTT = false;

// HTTP endpoint (if using HTTP)
const char* HTTP_ENDPOINT = "http://example.com/telemetry"; // replace

// MQTT (if using MQTT)
const char* MQTT_SERVER = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
const char* MQTT_TOPIC = "structmon/bridge1";

///////////////////// PINOUT /////////////////////
const int HX711_DOUT = 22; // DT
const int HX711_SCK  = 21; // SCK

// ADXL335 analog pins (ADC1)
const int AX_PIN = 34;
const int AY_PIN = 35;
const int AZ_PIN = 32;

const int LED_PIN = 2; // status

// ---------- HX711 & ADC setup ----------
HX711 scale;

// Calibration (after you calibrate these physically):
long hx_offset = 0;          // HX711 tare offset (set at zero)
float hx_scale = 1000.0f;    // scale factor: reading/known_unit => adjust after calibration

// ADXL calibration:
float zero_g_x = 1.65;   // volts at 0g, measure and replace (typically ~Vcc/2)
float zero_g_y = 1.65;
float zero_g_z = 1.65;
float sens_v_per_g = 0.300; // volts per g for ADXL335 (typical) - measure exact value

// Filtering and detection
const int HX_SAMPLES = 10;
const int ADXL_SAMPLES = 20;
float hx_ema = 0.0f;      // exponential moving average for strain reading
const float HX_EMA_ALPHA = 0.25f;
float accel_ema[3] = {0,0,0};
const float ACC_EMA_ALPHA = 0.15f;

// Event thresholds (tune these)
const float STRAIN_DELTA_THRESHOLD = 0.05f; // in units after scaling (e.g., microstrain or weight units)
const float VINSTANT_CHANGE_THRESHOLD = 0.2f; // g change for crack/vibration event
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 30*1000UL; // send every 30 s

// WiFi / MQTT objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ---------------- Helpers ----------------
float readADXLVoltage(int pin) {
  // ADC resolution 12-bit (0-4095), attenuation configured by default; use ADC calibration if needed
  int raw = analogRead(pin);
  float v = (raw / 4095.0f) * 3.3f; // change 3.3 if using other reference
  return v;
}

float adxl_voltage_to_g(float v, float zero_v) {
  return (v - zero_v) / sens_v_per_g;
}

long readHxAverage(int samples) {
  long sum = 0;
  for (int i=0;i<samples;i++){
    while (!scale.is_ready()) delay(1);
    sum += scale.read();
  }
  return sum / samples;
}

void connectWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  digitalWrite(LED_PIN, LOW);
  Serial.print("WiFi connecting");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000UL) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("WiFi failed");
  }
}

void connectMQTT(){
  if (!useMQTT) return;
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  while (!mqttClient.connected()){
    Serial.print("Connecting MQTT...");
    if (mqttClient.connect("ESP32_structMon")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2s");
      delay(2000);
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ADC resolution & attenuation for ADXL (make sure ADC pins support input)
  analogReadResolution(12); // 0..4095
  // Optionally set attenuation if needed:
  // analogSetPinAttenuation(AX_PIN, ADC_11db); etc. (available depending on core version)

  // HX711 init
  scale.begin(HX711_DOUT, HX711_SCK);
  // Give time for HX711 to stabilise
  delay(500);
  if (!scale.is_ready()) {
    Serial.println("HX711 NOT ready - check wiring");
  } else {
    Serial.println("HX711 ready");
  }

  // Tare / offset reading (fast tare)
  long tare = readHxAverage(HX_SAMPLES);
  hx_offset = tare;
  Serial.print("HX711 tare offset: "); Serial.println(hx_offset);

  // Initial EMA = initial reading
  hx_ema = (float)hx_offset;
  float vx = readADXLVoltage(AX_PIN); accel_ema[0] = adxl_voltage_to_g(vx, zero_g_x);
  float vy = readADXLVoltage(AY_PIN); accel_ema[1] = adxl_voltage_to_g(vy, zero_g_y);
  float vz = readADXLVoltage(AZ_PIN); accel_ema[2] = adxl_voltage_to_g(vz, zero_g_z);

  // Connect WiFi
  connectWiFi();
  if (useMQTT) connectMQTT();

  digitalWrite(LED_PIN, HIGH);
  Serial.println("Setup complete.");
}

// ---------------- Main loop ----------------
void loop() {
  // Maintain WiFi/MQTT
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (useMQTT && !mqttClient.connected()) connectMQTT();
  if (useMQTT) mqttClient.loop();

  // Read HX711 (strain)
  long raw = readHxAverage(HX_SAMPLES);
  float reading = (float)raw;
  // convert to "units" using hx_offset & hx_scale:
  float strain_unit = (reading - (float)hx_offset) / hx_scale; // e.g., microstrain, kg etc depending on your calibration
  // EMA smoothing
  hx_ema = HX_EMA_ALPHA * strain_unit + (1.0f - HX_EMA_ALPHA) * hx_ema;

  // Read ADXL (vibration) average
  float vx = 0, vy = 0, vz = 0;
  for (int i=0;i<ADXL_SAMPLES;i++){
    vx += readADXLVoltage(AX_PIN);
    vy += readADXLVoltage(AY_PIN);
    vz += readADXLVoltage(AZ_PIN);
    delay(2);
  }
  vx /= (float)ADXL_SAMPLES;
  vy /= (float)ADXL_SAMPLES;
  vz /= (float)ADXL_SAMPLES;
  float gx = adxl_voltage_to_g(vx, zero_g_x);
  float gy = adxl_voltage_to_g(vy, zero_g_y);
  float gz = adxl_voltage_to_g(vz, zero_g_z);

  // EMA filters for g
  accel_ema[0] = ACC_EMA_ALPHA * gx + (1.0f-ACC_EMA_ALPHA) * accel_ema[0];
  accel_ema[1] = ACC_EMA_ALPHA * gy + (1.0f-ACC_EMA_ALPHA) * accel_ema[1];
  accel_ema[2] = ACC_EMA_ALPHA * gz + (1.0f-ACC_EMA_ALPHA) * accel_ema[2];

  // Detect sudden events (crack or impulse)
  static float last_hx = 0.0f;
  static float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
  bool event_strain = fabs(hx_ema - last_hx) > STRAIN_DELTA_THRESHOLD;
  bool event_vib = (fabs(accel_ema[0]-last_gx) > VINSTANT_CHANGE_THRESHOLD)
                || (fabs(accel_ema[1]-last_gy) > VINSTANT_CHANGE_THRESHOLD)
                || (fabs(accel_ema[2]-last_gz) > VINSTANT_CHANGE_THRESHOLD);

  last_hx = hx_ema;
  last_gx = accel_ema[0]; last_gy = accel_ema[1]; last_gz = accel_ema[2];

  // Prepare JSON payload
  char payload[512];
  unsigned long t = millis();
  snprintf(payload, sizeof(payload),
    "{\"ts\":%lu,\"strain\":%.5f,\"strain_ema\":%.5f,\"accel_g\":[%.4f,%.4f,%.4f],\"accel_ema\":[%.4f,%.4f,%.4f],\"evt_strain\":%d,\"evt_vib\":%d}",
    t,
    strain_unit, hx_ema,
    gx, gy, gz,
    accel_ema[0], accel_ema[1], accel_ema[2],
    event_strain ? 1 : 0,
    event_vib ? 1 : 0
  );

  Serial.println(payload);

  // Send periodically or immediately on event
  if (millis() - lastSend > SEND_INTERVAL_MS || event_strain || event_vib) {
    if (useHTTP) {
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(HTTP_ENDPOINT);
        http.addHeader("Content-Type", "application/json");
        int httpCode = http.POST((uint8_t*)payload, strlen(payload));
        if (httpCode > 0) {
          Serial.print("HTTP response: "); Serial.println(httpCode);
        } else {
          Serial.print("HTTP failed: "); Serial.println(http.errorToString(httpCode).c_str());
        }
        http.end();
      }
    }
    if (useMQTT && mqttClient.connected()) {
      mqttClient.publish(MQTT_TOPIC, payload);
    }
    lastSend = millis();
  }

  delay(200); // main loop pacing
}
