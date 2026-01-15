/*
 * ════════════════════════════════════════════════════════════════════
 *  SOLAR POWERED IoT SMART POWER OUTLET & SAFETY MONITORING SYSTEM
 *  Complete MQTT Integration Test
 * ════════════════════════════════════════════════════════════════════
 * 
 * Components:
 *   - 4x Water Sensors (Digital)
 *   - 2x Voltage Sensors (Analog)
 *   - 2x Current Sensors (ACS712)
 *   - 2x Temperature Sensors (LM35)
 *   - 1x Gas Sensor (MQ Series)
 *   - 1x MPU6050 Gyroscope/Accelerometer
 *   - 4x Servo Motors (Breaker Control)
 *   - 1x Buzzer
 * 
 * MQTT Broker: HiveMQ Cloud (TLS/SSL)
 * 
 * ════════════════════════════════════════════════════════════════════
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <ACS712.h>
#include <ArduinoJson.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════════════════
//                         PIN CONFIGURATION
// ═══════════════════════════════════════════════════════════════════

// Water Sensors (Digital Output)
#define WATER1_DO  15
#define WATER2_DO  2
#define WATER3_DO  4
#define WATER4_DO  5

// Voltage Sensors (ADC1 - WiFi Safe)
#define VOLTAGE_SENSOR_1  32
#define VOLTAGE_SENSOR_2  33

// Current Sensors (ADC1 - WiFi Safe)
#define CURRENT_SENSOR_1  34
#define CURRENT_SENSOR_2  35

// Gas Sensor
#define GAS_SENSOR  25

// Temperature Sensors
#define TEMP_SENSOR_1  26
#define TEMP_SENSOR_2  27

// MPU6050 (I2C)
#define MPU_SDA  21
#define MPU_SCL  22
#define MPU_ADDR 0x68

// Servo Motors
#define SERVO_GYRO_1     16
#define SERVO_GYRO_2     17
#define BREAKER_SERVO_1  18
#define BREAKER_SERVO_2  23

// Buzzer
#define BUZZER  19

// ═══════════════════════════════════════════════════════════════════
//                         WiFi & MQTT SETTINGS
// ═══════════════════════════════════════════════════════════════════

const char* ssid = "wisa";
const char* password = "luisa1234";

const char* mqtt_server = "675a04533d074dc28220b4435ceb3a3b.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "hivemq.webclient.1767790067898";
const char* mqtt_password = ";m:X,hUd!306uOq1xNPH";
const char* client_id = "ESP32_SmartOutlet";

// ═══════════════════════════════════════════════════════════════════
//                         MQTT TOPICS
// ═══════════════════════════════════════════════════════════════════

// Publish Topics (ESP32 → Server)
const char* topic_voltage = "esp32/sensors/voltage";
const char* topic_current = "esp32/sensors/current";
const char* topic_temperature = "esp32/sensors/temperature";
const char* topic_gas = "esp32/sensors/gas";
const char* topic_water = "esp32/sensors/water";
const char* topic_motion = "esp32/sensors/motion";
const char* topic_system_status = "esp32/system/status";
const char* topic_alerts = "esp32/alerts";

// Subscribe Topics (Server → ESP32)
const char* topic_breaker_control = "esp32/control/breaker";
const char* topic_servo_control = "esp32/control/servo";
const char* topic_buzzer_control = "esp32/control/buzzer";
const char* topic_system_command = "esp32/control/system";

// ═══════════════════════════════════════════════════════════════════
//                         CONSTANTS & THRESHOLDS
// ═══════════════════════════════════════════════════════════════════

// Voltage Divider
const float R1 = 27000.0;
const float R2 = 10000.0;
const float ADC_REF = 3.3;
const int ADC_RES = 4095;

// Temperature
const float MV_PER_C = 0.01;
const float TEMP_WARNING = 45.0;
const float TEMP_CRITICAL = 60.0;

// Gas
const int GAS_WARNING = 2000;
const int GAS_DANGER = 3000;

// Current
const float OVERCURRENT_LIMIT = 4.0;

// Motion
const float SHAKE_THRESHOLD = 0.3;
const float EARTHQUAKE_THRESHOLD = 0.8;

// System
const float SYSTEM_VOLTAGE = 12.0;
const int BREAKER_ON = 90;
const int BREAKER_OFF = 0;

// ═══════════════════════════════════════════════════════════════════
//                         GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

// Servo objects
Servo servoGyro1;
Servo servoGyro2;
Servo breakerServo1;
Servo breakerServo2;

// Current sensor objects (ACS712-05B: 185mV/A)
ACS712 currentSensor1(CURRENT_SENSOR_1, 3.3, 4095, 185);
ACS712 currentSensor2(CURRENT_SENSOR_2, 3.3, 4095, 185);

// ═══════════════════════════════════════════════════════════════════
//                         GLOBAL VARIABLES
// ═══════════════════════════════════════════════════════════════════

// Sensor readings
float voltage1 = 0, voltage2 = 0;
float current1 = 0, current2 = 0;
float temp1 = 0, temp2 = 0;
int gasValue = 0;
bool water1 = false, water2 = false, water3 = false, water4 = false;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float totalAccel = 0;
float mpuTemp = 0;

// MPU6050 calibration
float axOff = 0, ayOff = 0, azOff = 0;
float gxOff = 0, gyOff = 0, gzOff = 0;

// System status
bool breakersOn = true;
bool systemArmed = true;
bool gasWarmedUp = false;
int gasBaseline = 0;
unsigned long gasStartTime = 0;

// Servo positions
int servoPos1 = 90, servoPos2 = 90, servoPos3 = 90, servoPos4 = 90;

// Timing
unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long SENSOR_INTERVAL = 500;      // Read sensors every 500ms
const unsigned long PUBLISH_INTERVAL = 2000;    // Publish every 2 seconds
const unsigned long RECONNECT_INTERVAL = 5000;  // Reconnect attempt every 5 seconds

// ═══════════════════════════════════════════════════════════════════
//                         ROOT CA CERTIFICATE
// ═══════════════════════════════════════════════════════════════════

// HiveMQ Cloud Root CA Certificate
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// ═══════════════════════════════════════════════════════════════════
//                         SETUP FUNCTION
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  printHeader();
  
  // Initialize pins
  initializePins();
  
  // Read ADC2 sensors BEFORE WiFi (ADC2 doesn't work with WiFi)
  readADC2Sensors();
  
  // Initialize I2C and MPU6050
  initializeMPU6050();
  
  // Initialize servos
  initializeServos();
  
  // Calibrate current sensors
  calibrateCurrentSensors();
  
  // Initialize buzzer
  initializeBuzzer();
  
  // Connect to WiFi
  connectWiFi();
  
  // Setup MQTT
  setupMQTT();
  
  // Record gas sensor start time
  gasStartTime = millis();
  
  Serial.println("\n✅ System initialization complete!");
  Serial.println("════════════════════════════════════════════\n");
  
  // Startup beep
  playTone(1000, 100);
  delay(100);
  playTone(1500, 100);
}

// ═══════════════════════════════════════════════════════════════════
//                         MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
  // Maintain MQTT connection
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();
  
  // Read sensors periodically
  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    readAllSensors();
    checkAlerts();
    lastSensorRead = millis();
  }
  
  // Publish to MQTT periodically
  if (millis() - lastMqttPublish >= PUBLISH_INTERVAL) {
    publishAllData();
    lastMqttPublish = millis();
  }
  
  // Check gas warmup
  if (!gasWarmedUp && (millis() - gasStartTime > 180000)) {
    gasWarmedUp = true;
    gasBaseline = gasValue;
    Serial.println("✅ Gas sensor warmed up! Baseline: " + String(gasBaseline));
  }
}

// ═══════════════════════════════════════════════════════════════════
//                         INITIALIZATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void printHeader() {
  Serial.println("\n");
  Serial.println("╔════════════════════════════════════════════════════════════╗");
  Serial.println("║  SOLAR POWERED IoT SMART POWER OUTLET                      ║");
  Serial.println("║  & SAFETY MONITORING SYSTEM                                ║");
  Serial.println("║  ────────────────────────────────────────────────────────  ║");
  Serial.println("║  MQTT Integration Test                                     ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝");
  Serial.println();
}

void initializePins() {
  Serial.println("📌 Initializing pins...");
  
  // Water sensors (Digital Input)
  pinMode(WATER1_DO, INPUT);
  pinMode(WATER2_DO, INPUT);
  pinMode(WATER3_DO, INPUT);
  pinMode(WATER4_DO, INPUT);
  
  // ADC settings
  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_SENSOR_1, ADC_11db);
  analogSetPinAttenuation(VOLTAGE_SENSOR_2, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR_1, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR_2, ADC_11db);
  analogSetPinAttenuation(GAS_SENSOR, ADC_11db);
  analogSetPinAttenuation(TEMP_SENSOR_1, ADC_11db);
  analogSetPinAttenuation(TEMP_SENSOR_2, ADC_11db);
  
  Serial.println("   ✓ Pins initialized");
}

void readADC2Sensors() {
  Serial.println("📊 Pre-reading ADC2 sensors (before WiFi)...");
  
  // Read ADC2 pins before WiFi starts
  for (int i = 0; i < 10; i++) {
    analogRead(GAS_SENSOR);
    analogRead(TEMP_SENSOR_1);
    analogRead(TEMP_SENSOR_2);
    delay(10);
  }
  
  Serial.println("   ✓ ADC2 pre-read complete");
}

void initializeMPU6050() {
  Serial.println("🔄 Initializing MPU6050...");
  
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);
  
  // Check connection
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  
  if (Wire.available()) {
    uint8_t deviceId = Wire.read();
    if (deviceId == 0x68 || deviceId == 0x98) {
      Serial.println("   ✓ MPU6050 connected (ID: 0x" + String(deviceId, HEX) + ")");
    } else {
      Serial.println("   ⚠️ Unknown device ID: 0x" + String(deviceId, HEX));
    }
  } else {
    Serial.println("   ❌ MPU6050 not found!");
    return;
  }
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  // Set accelerometer to ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Set gyroscope to ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Calibrate
  Serial.println("   Calibrating... Keep sensor still!");
  delay(1000);
  calibrateMPU6050();
  Serial.println("   ✓ MPU6050 calibrated");
}

void calibrateMPU6050() {
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;
  
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050Raw(ax, ay, az, gx, gy, gz);
    axSum += ax; aySum += ay; azSum += az;
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(10);
  }
  
  axOff = axSum / 100.0 / 16384.0;
  ayOff = aySum / 100.0 / 16384.0;
  azOff = (azSum / 100.0 / 16384.0) - 1.0;
  gxOff = gxSum / 100.0 / 131.0;
  gyOff = gySum / 100.0 / 131.0;
  gzOff = gzSum / 100.0 / 131.0;
}

void initializeServos() {
  Serial.println("🔧 Initializing servos...");
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  servoGyro1.attach(SERVO_GYRO_1, 500, 2400);
  servoGyro2.attach(SERVO_GYRO_2, 500, 2400);
  breakerServo1.attach(BREAKER_SERVO_1, 500, 2400);
  breakerServo2.attach(BREAKER_SERVO_2, 500, 2400);
  
  // Center all servos
  servoGyro1.write(90);
  servoGyro2.write(90);
  breakerServo1.write(BREAKER_ON);
  breakerServo2.write(BREAKER_ON);
  
  servoPos1 = servoPos2 = 90;
  servoPos3 = servoPos4 = BREAKER_ON;
  
  Serial.println("   ✓ Servos initialized");
}

void calibrateCurrentSensors() {
  Serial.println("⚡ Calibrating current sensors...");
  Serial.println("   (Disconnect loads for accurate calibration)");
  delay(1000);
  
  currentSensor1.autoMidPoint();
  currentSensor2.autoMidPoint();
  
  Serial.println("   ✓ Midpoint 1: " + String(currentSensor1.getMidPoint()));
  Serial.println("   ✓ Midpoint 2: " + String(currentSensor2.getMidPoint()));
}

void initializeBuzzer() {
  Serial.println("🔊 Initializing buzzer...");
  
  ledcSetup(0, 2000, 8);
  ledcAttachPin(BUZZER, 0);
  
  Serial.println("   ✓ Buzzer initialized");
}

// ═══════════════════════════════════════════════════════════════════
//                         WiFi FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void connectWiFi() {
  Serial.println("📶 Connecting to WiFi...");
  Serial.println("   SSID: " + String(ssid));
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("   ✓ Connected!");
    Serial.println("   IP: " + WiFi.localIP().toString());
    Serial.println("   RSSI: " + String(WiFi.RSSI()) + " dBm");
  } else {
    Serial.println();
    Serial.println("   ❌ Failed to connect!");
  }
}

// ═══════════════════════════════════════════════════════════════════
//                         MQTT FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void setupMQTT() {
  Serial.println("🌐 Setting up MQTT...");
  Serial.println("   Server: " + String(mqtt_server));
  Serial.println("   Port: " + String(mqtt_port));
  
  espClient.setCACert(root_ca);
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);
  
  Serial.println("   ✓ MQTT configured");
}

void reconnectMQTT() {
  if (millis() - lastReconnectAttempt < RECONNECT_INTERVAL) {
    return;
  }
  lastReconnectAttempt = millis();
  
  Serial.println("🔄 Connecting to MQTT broker...");
  
  if (mqtt.connect(client_id, mqtt_username, mqtt_password)) {
    Serial.println("   ✓ Connected to MQTT broker!");
    
    // Subscribe to control topics
    mqtt.subscribe(topic_breaker_control);
    mqtt.subscribe(topic_servo_control);
    mqtt.subscribe(topic_buzzer_control);
    mqtt.subscribe(topic_system_command);
    
    Serial.println("   ✓ Subscribed to control topics");
    
    // Publish online status
    publishSystemStatus("online");
    
  } else {
    Serial.print("   ❌ Failed, rc=");
    Serial.println(mqtt.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("\n📨 MQTT Message Received:");
  Serial.println("   Topic: " + String(topic));
  Serial.println("   Message: " + message);
  
  // Parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("   ⚠️ JSON parse error!");
    return;
  }
  
  // Handle breaker control
  if (String(topic) == topic_breaker_control) {
    handleBreakerControl(doc);
  }
  // Handle servo control
  else if (String(topic) == topic_servo_control) {
    handleServoControl(doc);
  }
  // Handle buzzer control
  else if (String(topic) == topic_buzzer_control) {
    handleBuzzerControl(doc);
  }
  // Handle system commands
  else if (String(topic) == topic_system_command) {
    handleSystemCommand(doc);
  }
}

// ═══════════════════════════════════════════════════════════════════
//                         CONTROL HANDLERS
// ═══════════════════════════════════════════════════════════════════

void handleBreakerControl(JsonDocument& doc) {
  if (doc.containsKey("action")) {
    String action = doc["action"].as<String>();
    
    if (action == "trip" || action == "off") {
      tripBreakers();
      publishAlert("breaker", "Breakers tripped via MQTT command");
    }
    else if (action == "reset" || action == "on") {
      resetBreakers();
      publishAlert("breaker", "Breakers reset via MQTT command");
    }
  }
}

void handleServoControl(JsonDocument& doc) {
  if (doc.containsKey("servo") && doc.containsKey("angle")) {
    int servoNum = doc["servo"].as<int>();
    int angle = doc["angle"].as<int>();
    
    angle = constrain(angle, 0, 180);
    
    switch (servoNum) {
      case 1: servoGyro1.write(angle); servoPos1 = angle; break;
      case 2: servoGyro2.write(angle); servoPos2 = angle; break;
      case 3: breakerServo1.write(angle); servoPos3 = angle; break;
      case 4: breakerServo2.write(angle); servoPos4 = angle; break;
    }
    
    Serial.println("   Servo " + String(servoNum) + " → " + String(angle) + "°");
  }
}

void handleBuzzerControl(JsonDocument& doc) {
  if (doc.containsKey("action")) {
    String action = doc["action"].as<String>();
    
    if (action == "beep") {
      int freq = doc["freq"] | 1000;
      int duration = doc["duration"] | 100;
      playTone(freq, duration);
    }
    else if (action == "alarm") {
      playAlarm();
    }
    else if (action == "stop") {
      ledcWriteTone(0, 0);
    }
  }
}

void handleSystemCommand(JsonDocument& doc) {
  if (doc.containsKey("command")) {
    String command = doc["command"].as<String>();
    
    if (command == "arm") {
      systemArmed = true;
      publishAlert("system", "System armed");
    }
    else if (command == "disarm") {
      systemArmed = false;
      publishAlert("system", "System disarmed");
    }
    else if (command == "status") {
      publishSystemStatus("online");
    }
    else if (command == "restart") {
      publishAlert("system", "Restarting...");
      delay(1000);
      ESP.restart();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//                         SENSOR READING FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void readAllSensors() {
  readVoltageSensors();
  readCurrentSensors();
  readTemperatureSensors();
  readGasSensor();
  readWaterSensors();
  readMPU6050();
}

void readVoltageSensors() {
  long sum1 = 0, sum2 = 0;
  
  for (int i = 0; i < 10; i++) {
    sum1 += analogRead(VOLTAGE_SENSOR_1);
    sum2 += analogRead(VOLTAGE_SENSOR_2);
    delayMicroseconds(100);
  }
  
  float avg1 = sum1 / 10.0;
  float avg2 = sum2 / 10.0;
  
  float adcV1 = (avg1 * ADC_REF) / ADC_RES;
  float adcV2 = (avg2 * ADC_REF) / ADC_RES;
  
  voltage1 = adcV1 * ((R1 + R2) / R2);
  voltage2 = adcV2 * ((R1 + R2) / R2);
}

void readCurrentSensors() {
  current1 = currentSensor1.mA_DC() / 1000.0;
  current2 = currentSensor2.mA_DC() / 1000.0;
}

void readTemperatureSensors() {
  long sum1 = 0, sum2 = 0;
  
  for (int i = 0; i < 10; i++) {
    sum1 += analogRead(TEMP_SENSOR_1);
    sum2 += analogRead(TEMP_SENSOR_2);
    delay(2);
  }
  
  float v1 = (sum1 / 10.0) * (ADC_REF / ADC_RES);
  float v2 = (sum2 / 10.0) * (ADC_REF / ADC_RES);
  
  temp1 = v1 / MV_PER_C;
  temp2 = v2 / MV_PER_C;
}

void readGasSensor() {
  long sum = 0;
  
  for (int i = 0; i < 10; i++) {
    sum += analogRead(GAS_SENSOR);
    delay(2);
  }
  
  gasValue = sum / 10;
}

void readWaterSensors() {
  // Inverted logic: LOW = water detected
  water1 = !digitalRead(WATER1_DO);
  water2 = !digitalRead(WATER2_DO);
  water3 = !digitalRead(WATER3_DO);
  water4 = !digitalRead(WATER4_DO);
}

void readMPU6050Raw(int16_t &ax, int16_t &ay, int16_t &az, 
                    int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
  
  mpuTemp = temp / 340.0 + 36.53;
}

void readMPU6050() {
  int16_t ax, ay, az, gx, gy, gz;
  readMPU6050Raw(ax, ay, az, gx, gy, gz);
  
  accelX = (ax / 16384.0) - axOff;
  accelY = (ay / 16384.0) - ayOff;
  accelZ = (az / 16384.0) - azOff;
  
  gyroX = (gx / 131.0) - gxOff;
  gyroY = (gy / 131.0) - gyOff;
  gyroZ = (gz / 131.0) - gzOff;
  
  totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
}

// ═══════════════════════════════════════════════════════════════════
//                         ALERT CHECKING
// ═══════════════════════════════════════════════════════════════════

void checkAlerts() {
  if (!systemArmed) return;
  
  // Check overcurrent
  if (abs(current1) > OVERCURRENT_LIMIT || abs(current2) > OVERCURRENT_LIMIT) {
    publishAlert("overcurrent", "Overcurrent detected! I1=" + String(current1, 2) + 
                 "A, I2=" + String(current2, 2) + "A");
    tripBreakers();
    playAlarm();
  }
  
  // Check temperature
  if (temp1 > TEMP_CRITICAL || temp2 > TEMP_CRITICAL) {
    publishAlert("temperature", "Critical temperature! T1=" + String(temp1, 1) + 
                 "°C, T2=" + String(temp2, 1) + "°C");
    tripBreakers();
    playAlarm();
  }
  
  // Check gas
  if (gasWarmedUp && gasValue > GAS_DANGER) {
    publishAlert("gas", "Dangerous gas level: " + String(gasValue));
    tripBreakers();
    playAlarm();
  }
  
  // Check water
  if (water1 || water2 || water3 || water4) {
    String waterStatus = "Water detected: ";
    if (water1) waterStatus += "S1 ";
    if (water2) waterStatus += "S2 ";
    if (water3) waterStatus += "S3 ";
    if (water4) waterStatus += "S4";
    publishAlert("water", waterStatus);
    tripBreakers();
    playAlarm();
  }
  
  // Check earthquake
  float deviation = abs(totalAccel - 1.0);
  if (deviation > EARTHQUAKE_THRESHOLD) {
    publishAlert("earthquake", "Earthquake detected! Deviation: " + String(deviation, 3) + "g");
    tripBreakers();
    playAlarm();
  }
}

// ═══════════════════════════════════════════════════════════════════
//                         MQTT PUBLISHING FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void publishAllData() {
  publishVoltageData();
  publishCurrentData();
  publishTemperatureData();
  publishGasData();
  publishWaterData();
  publishMotionData();
  publishSystemStatus("online");
  
  // Print to Serial for debugging
  printSensorReadings();
}

void publishVoltageData() {
  StaticJsonDocument<200> doc;
  doc["v1"] = round(voltage1 * 100) / 100.0;
  doc["v2"] = round(voltage2 * 100) / 100.0;
  doc["unit"] = "V";
  doc["timestamp"] = millis();
  
  char buffer[200];
  serializeJson(doc, buffer);
  mqtt.publish(topic_voltage, buffer);
}

void publishCurrentData() {
  StaticJsonDocument<256> doc;
  doc["i1"] = round(current1 * 1000) / 1000.0;
  doc["i2"] = round(current2 * 1000) / 1000.0;
  doc["p1"] = round(abs(current1) * SYSTEM_VOLTAGE * 100) / 100.0;
  doc["p2"] = round(abs(current2) * SYSTEM_VOLTAGE * 100) / 100.0;
  doc["total_power"] = round((abs(current1) + abs(current2)) * SYSTEM_VOLTAGE * 100) / 100.0;
  doc["unit_i"] = "A";
  doc["unit_p"] = "W";
  doc["timestamp"] = millis();
  
  char buffer[256];
  serializeJson(doc, buffer);
  mqtt.publish(topic_current, buffer);
}

void publishTemperatureData() {
  StaticJsonDocument<200> doc;
  doc["t1"] = round(temp1 * 10) / 10.0;
  doc["t2"] = round(temp2 * 10) / 10.0;
  doc["avg"] = round((temp1 + temp2) / 2.0 * 10) / 10.0;
  doc["unit"] = "C";
  doc["warning"] = (temp1 > TEMP_WARNING || temp2 > TEMP_WARNING);
  doc["timestamp"] = millis();
  
  char buffer[200];
  serializeJson(doc, buffer);
  mqtt.publish(topic_temperature, buffer);
}

void publishGasData() {
  StaticJsonDocument<200> doc;
  doc["value"] = gasValue;
  doc["baseline"] = gasBaseline;
  doc["deviation"] = gasValue - gasBaseline;
  doc["warmed_up"] = gasWarmedUp;
  doc["status"] = getGasStatus();
  doc["timestamp"] = millis();
  
  char buffer[200];
  serializeJson(doc, buffer);
  mqtt.publish(topic_gas, buffer);
}

void publishWaterData() {
  StaticJsonDocument<200> doc;
  doc["s1"] = water1;
  doc["s2"] = water2;
  doc["s3"] = water3;
  doc["s4"] = water4;
  doc["count"] = (int)water1 + (int)water2 + (int)water3 + (int)water4;
  doc["alert"] = (water1 || water2 || water3 || water4);
  doc["timestamp"] = millis();
  
  char buffer[200];
  serializeJson(doc, buffer);
  mqtt.publish(topic_water, buffer);
}

void publishMotionData() {
  StaticJsonDocument<300> doc;
  
  JsonObject accel = doc.createNestedObject("accel");
  accel["x"] = round(accelX * 100) / 100.0;
  accel["y"] = round(accelY * 100) / 100.0;
  accel["z"] = round(accelZ * 100) / 100.0;
  accel["total"] = round(totalAccel * 100) / 100.0;
  
  JsonObject gyro = doc.createNestedObject("gyro");
  gyro["x"] = round(gyroX * 10) / 10.0;
  gyro["y"] = round(gyroY * 10) / 10.0;
  gyro["z"] = round(gyroZ * 10) / 10.0;
  
  float deviation = abs(totalAccel - 1.0);
  doc["deviation"] = round(deviation * 1000) / 1000.0;
  doc["stable"] = (deviation < SHAKE_THRESHOLD);
  doc["earthquake"] = (deviation > EARTHQUAKE_THRESHOLD);
  doc["timestamp"] = millis();
  
  char buffer[300];
  serializeJson(doc, buffer);
  mqtt.publish(topic_motion, buffer);
}

void publishSystemStatus(String status) {
  StaticJsonDocument<400> doc;
  
  doc["status"] = status;
  doc["armed"] = systemArmed;
  doc["breakers_on"] = breakersOn;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  
  JsonObject servos = doc.createNestedObject("servos");
  servos["s1"] = servoPos1;
  servos["s2"] = servoPos2;
  servos["s3"] = servoPos3;
  servos["s4"] = servoPos4;
  
  doc["timestamp"] = millis();
  
  char buffer[400];
  serializeJson(doc, buffer);
  mqtt.publish(topic_system_status, buffer);
}

void publishAlert(String type, String message) {
  StaticJsonDocument<300> doc;
  doc["type"] = type;
  doc["message"] = message;
  doc["timestamp"] = millis();
  doc["armed"] = systemArmed;
  
  char buffer[300];
  serializeJson(doc, buffer);
  mqtt.publish(topic_alerts, buffer);
  
  Serial.println("\n🚨 ALERT: [" + type + "] " + message);
}

// ═══════════════════════════════════════════════════════════════════
//                         BREAKER CONTROL FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void tripBreakers() {
  Serial.println("\n⚠️ TRIPPING BREAKERS!");
  
  for (int i = servoPos3; i >= BREAKER_OFF; i -= 5) {
    breakerServo1.write(i);
    breakerServo2.write(i);
    delay(20);
  }
  
  servoPos3 = servoPos4 = BREAKER_OFF;
  breakersOn = false;
  
  Serial.println("🔴 BREAKERS TRIPPED - Power Disconnected");
}

void resetBreakers() {
  Serial.println("\n🔄 Resetting breakers...");
  
  for (int i = servoPos3; i <= BREAKER_ON; i += 3) {
    breakerServo1.write(i);
    breakerServo2.write(i);
    delay(30);
  }
  
  servoPos3 = servoPos4 = BREAKER_ON;
  breakersOn = true;
  
  Serial.println("✅ BREAKERS RESET - Power Connected");
}

// ═══════════════════════════════════════════════════════════════════
//                         BUZZER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

void playTone(int frequency, int duration) {
  ledcWriteTone(0, frequency);
  delay(duration);
  ledcWriteTone(0, 0);
}

void playAlarm() {
  for (int i = 0; i < 3; i++) {
    ledcWriteTone(0, 2500);
    delay(100);
    ledcWriteTone(0, 1500);
    delay(100);
  }
  ledcWriteTone(0, 0);
}

// ═══════════════════════════════════════════════════════════════════
//                         HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════

String getGasStatus() {
  if (!gasWarmedUp) return "warming_up";
  if (gasValue < 500) return "clean";
  if (gasValue < 1000) return "low";
  if (gasValue < 2000) return "moderate";
  if (gasValue < 3000) return "high";
  return "danger";
}

void printSensorReadings() {
  Serial.println("\n┌─────────────────────────────────────────────────────┐");
  Serial.println("│              SENSOR READINGS                        │");
  Serial.println("├─────────────────────────────────────────────────────┤");
  
  Serial.print("│ Voltage:  V1="); Serial.print(voltage1, 2);
  Serial.print("V  V2="); Serial.print(voltage2, 2); Serial.println("V              │");
  
  Serial.print("│ Current:  I1="); Serial.print(current1, 3);
  Serial.print("A  I2="); Serial.print(current2, 3); Serial.println("A            │");
  
  Serial.print("│ Temp:     T1="); Serial.print(temp1, 1);
  Serial.print("°C T2="); Serial.print(temp2, 1); Serial.println("°C              │");
  
  Serial.print("│ Gas:      "); Serial.print(gasValue);
  Serial.print(" ("); Serial.print(getGasStatus()); Serial.println(")                    │");
  
  Serial.print("│ Water:    [");
  Serial.print(water1 ? "W" : "-"); Serial.print("][");
  Serial.print(water2 ? "W" : "-"); Serial.print("][");
  Serial.print(water3 ? "W" : "-"); Serial.print("][");
  Serial.print(water4 ? "W" : "-"); Serial.println("]                         │");
  
  Serial.print("│ Motion:   "); Serial.print(totalAccel, 2);
  Serial.println("g                                 │");
  
  Serial.print("│ Breakers: "); 
  Serial.println(breakersOn ? "ON ✅                              │" : "OFF 🔴                             │");
  
  Serial.println("└─────────────────────────────────────────────────────┘");
}