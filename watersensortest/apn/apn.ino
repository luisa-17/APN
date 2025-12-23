#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// -------------------- WIFI CONFIGURATION --------------------
const char* ssid = "PLDTHOMEFIBR432a0";        
const char* password = "PLDTWIFIcv3e9";

// -------------------- MQTT CONFIGURATION --------------------
const char* mqtt_broker = "afa96665cdc74a5ca2cbef61c459704e.s1.eu.hivemq.cloud"; 
const int mqtt_port = 8883;
const char* mqtt_username = "esp32_apn";                 
const char* mqtt_password = "APN20250k";      

// HiveMQ Cloud Root CA Certificate (ISRG Root X1 - Let's Encrypt)
const char* root_ca = R"EOF(
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

// -------------------- DEVICE ID (from MAC address) --------------------
String deviceId;
String telemetryTopic;
String commandTopic;

// -------------------- PIN CONFIG --------------------
#define WATER1 32
#define WATER2 33
#define WATER3 34
#define WATER4 35

#define GAS_DO 27

#define TEMP1 25
#define TEMP2 26

#define VOLTAGE_SENSOR_1 14
#define VOLTAGE_SENSOR_2 12
#define CURRENT_SENSOR_1 13
#define CURRENT_SENSOR_2 15

#define BUZZER 19

#define SERVO_GYRO_1 16
#define SERVO_GYRO_2 17
#define BREAKER_SERVO_1 18
#define BREAKER_SERVO_2 23

// -------------------- THRESHOLDS --------------------
int waterThreshold = 500;
float tempThreshold = 50.0;
float gyroThreshold = 25.0;  // INCREASED from 15.0 to reduce sensitivity
float voltageThreshold = 3000;
float currentThreshold = 2000;

float tempWarningLevel = 40.0;
float voltageWarning = 2500;
float currentWarning = 1500;

// -------------------- SENSOR ENABLE FLAGS --------------------
bool enableVoltageSensors = false;
bool enableCurrentSensors = false;

// -------------------- SERVO CONFIG --------------------
Servo servoGyro1;
Servo servoGyro2;
Servo breakerServo1;
Servo breakerServo2;

int gyroRestAngle = 90;
int gyroShakeAngle1 = 45;
int gyroShakeAngle2 = 135;

int breakerOnAngle = 90;
int breakerOffAngle = 0;

bool breaker1State = true;
bool breaker2State = true;

unsigned long breaker1TripTime = 0;
unsigned long breaker2TripTime = 0;
const unsigned long autoReturnDelay = 1000;

// -------------------- GYRO FILTERING (NEW) --------------------
float gyroReadings[10] = {0};
int gyroIndex = 0;
unsigned long lastGyroTrigger = 0;
const unsigned long gyroCooldown = 2000;

// -------------------- STATE TRACKING --------------------
unsigned long lastAlertTime = 0;
unsigned long alertCooldown = 5000;
int alertCount = 0;
unsigned long statusInterval = 30000;
unsigned long lastStatusTime = 0;

bool waterAlertActive = false;
bool gasAlertActive = false;
bool tempAlertActive = false;
bool gyroAlertActive = false;
bool powerAlertActive = false;

// -------------------- STATE CHANGE TRACKING --------------------
bool prevWaterDetected = false;
bool prevGasDetected = false;
bool prevTempCritical = false;
bool prevTempWarning = false;
bool prevGyroDetected = false;
bool prevPowerCritical = false;
bool prevPowerWarning = false;

bool prevWifiConnected = false;
bool prevMqttConnected = false;
bool prevBreaker1State = true;
bool prevBreaker2State = true;
bool prevSystemEnabled = true;
bool prevBuzzerEnabled = true;

unsigned long lastHeartbeat = 0;
unsigned long heartbeatInterval = 60000;

unsigned long lastPeriodicUpdate = 0;
unsigned long periodicUpdateInterval = 30000;
bool enablePeriodicUpdates = true;

unsigned long lastWiFiCheck = 0;
bool wifiConnecting = false;

// -------------------- MPU OBJECT --------------------
MPU6050 mpu(Wire);

// -------------------- MQTT CLIENT --------------------
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
bool mqttConnected = false;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long mqttReconnectInterval = 5000;
unsigned long lastDataSendTime = 0;
unsigned long dataSendInterval = 2000;
bool systemEnabled = true;
bool buzzerEnabled = true;

// -------------------- JSON BUFFER --------------------
StaticJsonDocument<1024> jsonDoc;
StaticJsonDocument<512> jsonReceive;

// ===================================================================
// FORWARD DECLARATIONS
// ===================================================================
void sendAlert(String alertType, String details = "");
void sendAlertCleared(String alertType);
void sendSensorData(bool forceImmediate = false);
void sendSystemStatus();
void printWiFiInfo();
void scanNetworks();
void printSensorReadings();
void printFullStatus();
void printHelp();
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool mqttReconnect();
String getDeviceId();
void handleAlerts();
void tripBreaker(int breakerNum);
void resetBreaker(int breakerNum);
bool checkGyroMovement();

// ===================================================================
// DEVICE ID GENERATION
// ===================================================================
String getDeviceId() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[13];
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// ===================================================================
// GYRO FILTERING FUNCTION (NEW)
// ===================================================================
bool checkGyroMovement() {
  mpu.update();
  
  float gyroMovement = abs(mpu.getGyroX()) + abs(mpu.getGyroY()) + abs(mpu.getGyroZ());
  
  gyroReadings[gyroIndex] = gyroMovement;
  gyroIndex = (gyroIndex + 1) % 10;
  
  float avgMovement = 0;
  for(int i = 0; i < 10; i++) {
    avgMovement += gyroReadings[i];
  }
  avgMovement /= 10.0;
  
  unsigned long currentTime = millis();
  bool isMoving = avgMovement > gyroThreshold;
  
  if (isMoving && (currentTime - lastGyroTrigger > gyroCooldown)) {
    lastGyroTrigger = currentTime;
    return true;
  }
  
  return false;
}

// ===================================================================
// ZONE-SPECIFIC SENSOR CHECKS (NEW)
// ===================================================================
bool checkZone1Water() {
  return (analogRead(WATER1) > waterThreshold) || 
         (analogRead(WATER2) > waterThreshold);
}

bool checkZone2Water() {
  return (analogRead(WATER3) > waterThreshold) || 
         (analogRead(WATER4) > waterThreshold);
}

bool checkZone1Power() {
  if (!enableVoltageSensors && !enableCurrentSensors) return false;
  
  bool voltageIssue = enableVoltageSensors && 
                      (analogRead(VOLTAGE_SENSOR_1) > voltageThreshold);
  bool currentIssue = enableCurrentSensors && 
                      (analogRead(CURRENT_SENSOR_1) > currentThreshold);
  
  return voltageIssue || currentIssue;
}

bool checkZone2Power() {
  if (!enableVoltageSensors && !enableCurrentSensors) return false;
  
  bool voltageIssue = enableVoltageSensors && 
                      (analogRead(VOLTAGE_SENSOR_2) > voltageThreshold);
  bool currentIssue = enableCurrentSensors && 
                      (analogRead(CURRENT_SENSOR_2) > currentThreshold);
  
  return voltageIssue || currentIssue;
}

bool checkZone1Temp() {
  return readTemp(TEMP1) >= tempThreshold;
}

bool checkZone2Temp() {
  return readTemp(TEMP2) >= tempThreshold;
}

// ===================================================================
// FIXED TRIP BREAKER FUNCTION
// ===================================================================
void tripBreaker(int breakerNum) {
  Servo* servo;
  unsigned long* tripTimePtr;
  int servoPin;
  
  if (breakerNum == 1) {
    servo = &breakerServo1;
    tripTimePtr = &breaker1TripTime;
    servoPin = BREAKER_SERVO_1;
  } else {
    servo = &breakerServo2;
    tripTimePtr = &breaker2TripTime;
    servoPin = BREAKER_SERVO_2;
  }
  
  Serial.printf("\n🚨 TRIPPING BREAKER %d...\n", breakerNum);
  
  // Attach servo
  if (!servo->attached()) {
    servo->attach(servoPin);
    delay(100);
  }
  
  // Start at ready position
  servo->write(breakerOnAngle);
  delay(200);
  
  // Trip motion (fast push)
  Serial.printf("   Pushing breaker OFF (%d° → %d°)...\n", breakerOnAngle, breakerOffAngle);
  for (int pos = breakerOnAngle; pos >= breakerOffAngle; pos -= 10) {
    servo->write(pos);
    delay(8);
  }
  servo->write(breakerOffAngle);
  delay(300);
  
  Serial.printf("⚡ Breaker %d: Physical breaker PUSHED OFF\n", breakerNum);
  
  // Record trip time
  *tripTimePtr = millis();
  
  // Auto-return to ready position
  delay(500);
  Serial.printf("   Servo returning to ready position (%d° → %d°)...\n", breakerOffAngle, breakerOnAngle);
  for (int pos = breakerOffAngle; pos <= breakerOnAngle; pos += 5) {
    servo->write(pos);
    delay(15);
  }
  servo->write(breakerOnAngle);
  delay(200);
  
  // Detach to save power and prevent jitter
  servo->detach();
  
  Serial.printf("✅ Breaker %d: Servo ready for next trip\n", breakerNum);
  Serial.printf("👉 User must manually flip physical breaker to ON\n\n");
}


// ===================================================================
// SIMULTANEOUS TRIP ALL BREAKERS - FIXED FOR PARALLEL OPERATION
// ===================================================================
void tripAllBreakers() {
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║   🚨 EMERGENCY - TRIPPING ALL    ║");
  Serial.println("║        CIRCUIT BREAKERS!          ║");
  Serial.println("╚═══════════════════════════════════╝\n");
  
  // Attach both servos
  if (!breakerServo1.attached()) {
    breakerServo1.attach(BREAKER_SERVO_1);
  }
  if (!breakerServo2.attached()) {
    breakerServo2.attach(BREAKER_SERVO_2);
  }
  delay(100);
  
  // Move both to ready position
  breakerServo1.write(breakerOnAngle);
  breakerServo2.write(breakerOnAngle);
  delay(200);
  
  Serial.println("⚡ Tripping both breakers SIMULTANEOUSLY...");
  Serial.printf("   Pushing both OFF (%d° → %d°)...\n", breakerOnAngle, breakerOffAngle);
  
  // Trip both at the same time
  for (int pos = breakerOnAngle; pos >= breakerOffAngle; pos -= 10) {
    breakerServo1.write(pos);
    breakerServo2.write(pos);
    delay(8);
  }
  
  breakerServo1.write(breakerOffAngle);
  breakerServo2.write(breakerOffAngle);
  delay(300);
  
  Serial.println("⚡ Both physical breakers PUSHED OFF");
  
  // Record trip times
  unsigned long tripTime = millis();
  breaker1TripTime = tripTime;
  breaker2TripTime = tripTime;
  
  // Auto-return both to ready position
  delay(500);
  Serial.println("🔄 Both servos returning to ready position...");
  
  for (int pos = breakerOffAngle; pos <= breakerOnAngle; pos += 5) {
    breakerServo1.write(pos);
    breakerServo2.write(pos);
    delay(15);
  }
  
  breakerServo1.write(breakerOnAngle);
  breakerServo2.write(breakerOnAngle);
  delay(200);
  
  // Detach both servos
  breakerServo1.detach();
  breakerServo2.detach();
  
  Serial.println("✅ Both servos ready for next trip");
  Serial.println("👉 User must manually flip both physical breakers to ON\n");
}



// ===================================================================
// MAIN ALERT HANDLING FUNCTION (COMPLETELY REWRITTEN)
// ===================================================================
void handleAlerts() {
  unsigned long currentTime = millis();
  
  bool gasDetected = (digitalRead(GAS_DO) == LOW);
  bool gyroDetected = checkGyroMovement();
  
  bool zone1Water = checkZone1Water();
  bool zone2Water = checkZone2Water();
  bool zone1Power = checkZone1Power();
  bool zone2Power = checkZone2Power();
  bool zone1Temp = checkZone1Temp();
  bool zone2Temp = checkZone2Temp();
  
  bool zone1Hazard = zone1Water || zone1Power || zone1Temp;
  bool zone2Hazard = zone2Water || zone2Power || zone2Temp;
  
  // ==================== PRIORITY 1: EMERGENCY TRIP ALL ====================
  if (gasDetected || gyroDetected) {
    
    if (gasDetected) {
      buzzGas();
      if (!gasAlertActive) {
        Serial.println("\n🚨🚨🚨 GAS LEAK DETECTED 🚨🚨🚨");
        Serial.println("⚡ EMERGENCY: Tripping ALL breakers NOW!");
        sendAlert("GAS_LEAK_DETECTED");
        gasAlertActive = true;
        tripAllBreakers();  // Trip immediately on first detection
      }
    }
    
    if (gyroDetected) {
      buzzGyro();
      if (!gyroAlertActive) {
        Serial.println("\n🚨🚨🚨 EARTHQUAKE DETECTED 🚨🚨🚨");
        Serial.println("⚡ EMERGENCY: Tripping ALL breakers NOW!");
        sendAlert("GROUND_MOVEMENT_DETECTED");
        gyroAlertActive = true;
        tripAllBreakers();  // Trip immediately on first detection
      }
    }
    
    return;  // Exit - don't process zone alerts
  }
  
  // ==================== PRIORITY 2: ZONE 1 HAZARDS ====================
  if (zone1Hazard) {
    bool shouldTrip = false;
    
    if (zone1Water && !waterAlertActive) {
      buzzWater();
      Serial.println("\n🚨 ZONE 1: WATER DETECTED - TRIPPING BREAKER 1");
      sendAlert("WATER_DETECTED", "Zone 1");
      waterAlertActive = true;
      shouldTrip = true;
    }
    if (zone1Temp && !tempAlertActive) {
      buzzTemp();
      Serial.println("\n🚨 ZONE 1: HIGH TEMPERATURE - TRIPPING BREAKER 1");
      sendAlert("HIGH_TEMPERATURE", "Zone 1");
      tempAlertActive = true;
      shouldTrip = true;
    }
    if (zone1Power && !powerAlertActive) {
      buzzPower();
      Serial.println("\n🚨 ZONE 1: POWER ABNORMAL - TRIPPING BREAKER 1");
      sendAlert("POWER_ABNORMAL", "Zone 1");
      powerAlertActive = true;
      shouldTrip = true;
    }
    
    if (shouldTrip) {
      tripBreaker(1);  // Trip on first detection only
    }
  }
  
  // ==================== PRIORITY 3: ZONE 2 HAZARDS ====================
  if (zone2Hazard) {
    bool shouldTrip = false;
    
    if (zone2Water) {
      buzzWater();
      Serial.println("\n🚨 ZONE 2: WATER DETECTED - TRIPPING BREAKER 2");
      sendAlert("WATER_DETECTED", "Zone 2");
      shouldTrip = true;
    }
    if (zone2Temp) {
      buzzTemp();
      Serial.println("\n🚨 ZONE 2: HIGH TEMPERATURE - TRIPPING BREAKER 2");
      sendAlert("HIGH_TEMPERATURE", "Zone 2");
      shouldTrip = true;
    }
    if (zone2Power) {
      buzzPower();
      Serial.println("\n🚨 ZONE 2: POWER ABNORMAL - TRIPPING BREAKER 2");
      sendAlert("POWER_ABNORMAL", "Zone 2");
      shouldTrip = true;
    }
    
    if (shouldTrip) {
      tripBreaker(2);  // Trip on first detection only
    }
  }
  
  // ==================== CLEAR ALERTS ====================
  if (!gasDetected && gasAlertActive) {
    Serial.println("✅ CLEARED: Gas level normal");
    sendAlertCleared("GAS");
    gasAlertActive = false;
  }
  
  if (!gyroDetected && gyroAlertActive) {
    Serial.println("✅ CLEARED: Movement stopped");
    sendAlertCleared("MOVEMENT");
    gyroAlertActive = false;
  }
  
  if (!zone1Water && !zone2Water && waterAlertActive) {
    Serial.println("✅ CLEARED: Water level normal");
    sendAlertCleared("WATER");
    waterAlertActive = false;
  }
  
  if (!zone1Temp && !zone2Temp && tempAlertActive) {
    Serial.println("✅ CLEARED: Temperature normal");
    sendAlertCleared("TEMPERATURE");
    tempAlertActive = false;
  }
  
  if (!zone1Power && !zone2Power && powerAlertActive) {
    Serial.println("✅ CLEARED: Power normal");
    sendAlertCleared("POWER");
    powerAlertActive = false;
  }
  
  // Turn off buzzer when all clear
  if (!gasDetected && !gyroDetected && !zone1Hazard && !zone2Hazard) {
    digitalWrite(BUZZER, LOW);
  }
}

// ===================================================================
// MQTT FUNCTIONS
// ===================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.print("📨 MQTT Message on topic: ");
  Serial.println(topic);
  Serial.print("   Payload: ");
  Serial.println(message);
  
  DeserializationError error = deserializeJson(jsonReceive, message);
  
  if (error) {
    Serial.println("❌ Error parsing MQTT message");
    return;
  }
  
  handleIncomingCommand();
}

void handleIncomingCommand() {
  if (jsonReceive.containsKey("command")) {
    String command = jsonReceive["command"].as<String>();
    command.toUpperCase();
    
    Serial.println("\n╔═══════════════════════════════════╗");
    Serial.print("║  Command: ");
    Serial.print(command);
    
    for(int i = command.length(); i < 21; i++) Serial.print(" ");
    Serial.println("║");
    Serial.println("╚═══════════════════════════════════╝\n");
    
    if (command == "SYSTEM_ON") {
      systemEnabled = true;
      sendAckResponse("System enabled");
    } 
    else if (command == "SYSTEM_OFF") {
      systemEnabled = false;
      digitalWrite(BUZZER, LOW);
      sendAckResponse("System disabled");
    } 
    else if (command == "BUZZER_ON") {
      buzzerEnabled = true;
      sendAckResponse("Buzzer enabled");
    } 
    else if (command == "BUZZER_OFF") {
      buzzerEnabled = false;
      digitalWrite(BUZZER, LOW);
      sendAckResponse("Buzzer muted");
    } 
    else if (command == "REQUEST_STATUS") {
      sendSensorData(true);
    } 
    else if (command == "CALIBRATE_GYRO") {
      mpu.calcGyroOffsets(true);
      sendAckResponse("Gyro calibrated");
    } 
    else if (command == "RESET_SYSTEM") {
      sendAckResponse("System resetting");
      delay(500);
      ESP.restart();
    } 
    else if (command == "SHAKE_TEST") {
      testGyroShake();
      sendAckResponse("Shake test completed");
    }
    // BREAKER COMMANDS - TRIP ONLY (NO RESET)
    else if (command == "BREAKER1_OFF") {
      tripBreaker(1);
      sendAckResponse("Breaker 1 tripped OFF - user must manually reset");
    } 
    else if (command == "BREAKER2_OFF") {
      tripBreaker(2);
      sendAckResponse("Breaker 2 tripped OFF - user must manually reset");
    } 
    else if (command == "TRIP_ALL") {
      tripAllBreakers();
      sendAckResponse("All breakers tripped - user must manually reset");
    } 
    else if (command == "GET_BREAKER_STATUS") {
      sendBreakerStatus();
    } 
    else {
      Serial.println("⚠️ Unknown command: " + command);
    }
  }
  else if (jsonReceive.containsKey("set_thresholds")) {
    JsonObject thresholds = jsonReceive["set_thresholds"];
    
    if (thresholds.containsKey("water")) {
      waterThreshold = thresholds["water"];
    }
    if (thresholds.containsKey("temperature")) {
      tempThreshold = thresholds["temperature"];
    }
    if (thresholds.containsKey("gyro")) {
      gyroThreshold = thresholds["gyro"];
    }
    if (thresholds.containsKey("voltage")) {
      voltageThreshold = thresholds["voltage"];
    }
    if (thresholds.containsKey("current")) {
      currentThreshold = thresholds["current"];
    }
    
    sendAckResponse("Thresholds updated");
    sendSystemStatus();
  }
}

void sendBreakerStatus() {
  if (!mqttClient.connected()) return;
  
  jsonDoc.clear();
  jsonDoc["message_type"] = "BREAKER_STATUS";
  jsonDoc["device_id"] = deviceId;
  
  JsonObject b1 = jsonDoc.createNestedObject("breaker1");
  b1["state"] = breaker1State ? "ON" : "OFF";
  b1["circuit"] = breaker1State ? "CLOSED" : "OPEN";
  b1["last_trip"] = breaker1TripTime;
  
  JsonObject b2 = jsonDoc.createNestedObject("breaker2");
  b2["state"] = breaker2State ? "ON" : "OFF";
  b2["circuit"] = breaker2State ? "CLOSED" : "OPEN";
  b2["last_trip"] = breaker2TripTime;
  
  jsonDoc["timestamp"] = millis() / 1000;
  jsonDoc["type"] = "status_update";
  
  String output;
  serializeJson(jsonDoc, output);
  publishMessage(output.c_str());
  
  Serial.println("✅ Breaker status sent");
}

bool mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  String clientId = "ESP32-" + deviceId;
  
  Serial.print("📡 MQTT: Connecting to ");
  Serial.print(mqtt_broker);
  Serial.print(" as ");
  Serial.println(clientId);
  
  if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
    Serial.println("✅ MQTT: Connected!");
    
    mqttClient.subscribe(commandTopic.c_str(), 1);
    Serial.print("📬 MQTT: Subscribed to ");
    Serial.println(commandTopic);
    
    sendSystemStatus();
    
    return true;
  } else {
    Serial.print("❌ MQTT: Connection failed, rc=");
    Serial.println(mqttClient.state());
    return false;
  }
}

void publishMessage(const char* payload) {
  if (!mqttClient.connected()) {
    Serial.println("⚠️ MQTT: Not connected, cannot publish");
    return;
  }
  
  if (mqttClient.publish(telemetryTopic.c_str(), payload, false)) {
    Serial.println("📤 MQTT: Message published");
  } else {
    Serial.println("❌ MQTT: Publish failed");
  }
}

// ===================================================================
// SERVO CONTROL FUNCTIONS
// ===================================================================
void initServos() {
  servoGyro1.attach(SERVO_GYRO_1);
  servoGyro2.attach(SERVO_GYRO_2);
  breakerServo1.attach(BREAKER_SERVO_1);
  breakerServo2.attach(BREAKER_SERVO_2);
  
  servoGyro1.write(gyroRestAngle);
  servoGyro2.write(gyroRestAngle);
  breakerServo1.write(breakerOnAngle);
  breakerServo2.write(breakerOnAngle);
  
  delay(500);
}

void testGyroShake() {
  Serial.println("🔧 Starting 30-second gyro shake test...");
  
  if (!servoGyro1.attached()) {
    Serial.println("⚠️  Servo 1 not attached, reattaching...");
    servoGyro1.attach(SERVO_GYRO_1);
    delay(100);
  }
  if (!servoGyro2.attached()) {
    Serial.println("⚠️  Servo 2 not attached, reattaching...");
    servoGyro2.attach(SERVO_GYRO_2);
    delay(100);
  }
  
  servoGyro1.write(gyroRestAngle);
  servoGyro2.write(gyroRestAngle);
  delay(200);
  
  unsigned long shakeStartTime = millis();
  unsigned long shakeDuration = 30000;
  int cycleCount = 0;
  
  Serial.println("🔳 Servos activated - shaking for 30 seconds...");
  
  while (millis() - shakeStartTime < shakeDuration) {
    if (!servoGyro1.attached()) {
      servoGyro1.attach(SERVO_GYRO_1);
    }
    if (!servoGyro2.attached()) {
      servoGyro2.attach(SERVO_GYRO_2);
    }
    
    servoGyro1.write(gyroShakeAngle1);
    servoGyro2.write(gyroShakeAngle2);
    delay(150);
    
    servoGyro1.write(gyroShakeAngle2);
    servoGyro2.write(gyroShakeAngle1);
    delay(150);
    
    servoGyro1.write(gyroRestAngle);
    servoGyro2.write(gyroRestAngle);
    delay(100);
    
    cycleCount++;
    
    unsigned long elapsed = millis() - shakeStartTime;
    if (elapsed % 5000 < 400) {
      Serial.print("  ⏱️  ");
      Serial.print(elapsed / 1000);
      Serial.print("s elapsed (");
      Serial.print(cycleCount);
      Serial.println(" cycles)...");
    }
    
    mqttClient.loop();
    yield();
  }
  
  servoGyro1.write(gyroRestAngle);
  servoGyro2.write(gyroRestAngle);
  delay(200);
  
  Serial.print("✅ 30-second gyro test complete (");
  Serial.print(cycleCount);
  Serial.println(" cycles)");
}

// ===================================================================
// BUZZER PATTERNS
// ===================================================================
void buzzWater() {
  if (!buzzerEnabled) return;
  digitalWrite(BUZZER, HIGH);
  delay(300);
  digitalWrite(BUZZER, LOW);
  delay(300);
}

void buzzGas() {
  if (!buzzerEnabled) return;
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(100);
}

void buzzTemp() {
  if (!buzzerEnabled) return;
  digitalWrite(BUZZER, HIGH);
  delay(600);
  digitalWrite(BUZZER, LOW);
  delay(300);
}

void buzzGyro() {
  if (!buzzerEnabled) return;
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(120);
    digitalWrite(BUZZER, LOW);
    delay(120);
  }
}

void buzzPower() {
  if (!buzzerEnabled) return;
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(400);
    digitalWrite(BUZZER, LOW);
    delay(200);
  }
}

void buzzCritical() {
  if (!buzzerEnabled) return;
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(80);
    digitalWrite(BUZZER, LOW);
    delay(80);
  }
}

void buzzWarning() {
  if (!buzzerEnabled) return;
  digitalWrite(BUZZER, HIGH);
  delay(150);
  digitalWrite(BUZZER, LOW);
  delay(500);
}

// ===================================================================
// SENSOR READING FUNCTIONS
// ===================================================================
float readTemp(int pin) {
  int raw = analogRead(pin);
  return (raw * 3.3 / 4095.0) * 100;
}

float convertVoltage(int reading) {
  return (reading * 3.3 / 4095.0) * 5.0;
}

float convertCurrent(int reading) {
  float voltage = reading * 3.3 / 4095.0;
  return (voltage - 1.65) / 0.066;
}

// ===================================================================
// HELPER FUNCTIONS FOR SERIAL OUTPUT
// ===================================================================
void printWiFiInfo() {
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║         WiFi Information          ║");
  Serial.println("╠═══════════════════════════════════╣");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("║  Status: ✅ CONNECTED             ║");
    Serial.print("║  SSID: ");
    String ssidStr = WiFi.SSID();
    Serial.print(ssidStr);
    for(int i = ssidStr.length(); i < 24; i++) Serial.print(" ");
    Serial.println("║");
    Serial.print("║  IP: ");
    Serial.print(WiFi.localIP());
    Serial.println("            ║");
    Serial.print("║  Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm               ║");
    Serial.print("║  MQTT: ");
    Serial.print(mqttClient.connected() ? "✅ Connected" : "⏳ Waiting");
    Serial.println("           ║");
  } else {
    Serial.println("║  Status: ❌ DISCONNECTED          ║");
  }
  Serial.println("╚═══════════════════════════════════╝\n");
}

void scanNetworks() {
  Serial.println("\n🔍 Scanning for WiFi networks...\n");
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("   No networks found!");
  } else {
    for (int i = 0; i < n; i++) {
      Serial.print("   ");
      Serial.print(i + 1);
      Serial.print(". ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm) ");
      
      if (WiFi.SSID(i).indexOf("5G") >= 0 || WiFi.SSID(i).indexOf("5g") >= 0) {
        Serial.print("⚠️ 5GHz - NOT COMPATIBLE!");
      } else {
        Serial.print("✅ 2.4GHz");
      }
      Serial.println();
    }
    Serial.println("\n   ⚠️  ESP32 only supports 2.4GHz networks!\n");
  }
}

void printSensorReadings() {
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║       CURRENT SENSOR VALUES       ║");
  Serial.println("╠═══════════════════════════════════╣");
  
  int w1 = analogRead(WATER1);
  int w2 = analogRead(WATER2);
  int w3 = analogRead(WATER3);
  int w4 = analogRead(WATER4);
  Serial.printf("║  Water: %d, %d, %d, %d\n", w1, w2, w3, w4);
  Serial.printf("║         (threshold: %d)\n", waterThreshold);
  
  float t1 = readTemp(TEMP1);
  float t2 = readTemp(TEMP2);
  Serial.printf("║  Temp: %.1f°C, %.1f°C\n", t1, t2);
  Serial.printf("║        (threshold: %.1f°C)\n", tempThreshold);
  
  mpu.update();
  float gyro = abs(mpu.getGyroX()) + abs(mpu.getGyroY()) + abs(mpu.getGyroZ());
  Serial.printf("║  Gyro: %.2f\n", gyro);
  Serial.printf("║        (threshold: %.2f)\n", gyroThreshold);
  
  Serial.print("║  Gas: ");
  Serial.println((digitalRead(GAS_DO) == LOW) ? "⚠️ DETECTED" : "Normal");
  
  if (enableVoltageSensors || enableCurrentSensors) {
    Serial.println("║  Power:");
    if (enableVoltageSensors) {
      Serial.printf("║    V1=%d V2=%d\n", analogRead(VOLTAGE_SENSOR_1), analogRead(VOLTAGE_SENSOR_2));
    }
    if (enableCurrentSensors) {
      Serial.printf("║    C1=%d C2=%d\n", analogRead(CURRENT_SENSOR_1), analogRead(CURRENT_SENSOR_2));
    }
  }
  
  Serial.println("╚═══════════════════════════════════╝\n");
}

void printFullStatus() {
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║         SYSTEM STATUS             ║");
  Serial.println("╠═══════════════════════════════════╣");
  Serial.print("║  Device ID: ");
  Serial.print(deviceId);
  Serial.println("      ║");
  Serial.print("║  WiFi:    ");
  Serial.println((WiFi.status() == WL_CONNECTED) ? "✅ Connected          ║" : "❌ Disconnected       ║");
  Serial.print("║  MQTT:    ");
  Serial.println(mqttClient.connected() ? "✅ Connected          ║" : "⏳ Waiting            ║");
  Serial.print("║  System:  ");
  Serial.println(systemEnabled ? "✅ Enabled            ║" : "❌ Disabled           ║");
  Serial.print("║  Buzzer:  ");
  Serial.println(buzzerEnabled ? "🔊 Enabled            ║" : "🔇 Muted              ║");
  
  Serial.println("╠═══════════════════════════════════╣");
  Serial.println("║      CIRCUIT BREAKER STATUS       ║");
  Serial.println("╠═══════════════════════════════════╣");
  
  Serial.print("║  Breaker 1: ");
  if (breaker1State) {
    Serial.println("✅ ON (Circuit Closed) ║");
  } else {
    Serial.println("⚡ TRIPPED (Open)      ║");
    Serial.print("║    Last trip: ");
    Serial.print((millis() - breaker1TripTime) / 1000);
    Serial.println("s ago        ║");
  }
  
  Serial.print("║  Breaker 2: ");
  if (breaker2State) {
    Serial.println("✅ ON (Circuit Closed) ║");
  } else {
    Serial.println("⚡ TRIPPED (Open)      ║");
    Serial.print("║    Last trip: ");
    Serial.print((millis() - breaker2TripTime) / 1000);
    Serial.println("s ago        ║");
  }
  
  unsigned long uptime = millis() / 1000;
  Serial.println("╠═══════════════════════════════════╣");
  Serial.print("║  Uptime:  ");
  Serial.print(uptime / 3600); Serial.print("h ");
  Serial.print((uptime % 3600) / 60); Serial.print("m ");
  Serial.print(uptime % 60); Serial.println("s             ║");
  Serial.println("╠═══════════════════════════════════╣");
  Serial.println("║         ACTIVE ALERTS             ║");
  Serial.println("╠═══════════════════════════════════╣");
  
  bool anyAlert = false;
  if (waterAlertActive) { Serial.println("║  💧 Water detected              ║"); anyAlert = true; }
  if (gasAlertActive) { Serial.println("║  💨 Gas detected                ║"); anyAlert = true; }
  if (tempAlertActive) { Serial.println("║  🔥 High temperature            ║"); anyAlert = true; }
  if (gyroAlertActive) { Serial.println("║  🔳 Ground movement             ║"); anyAlert = true; }
  if (powerAlertActive) { Serial.println("║  ⚡ Power abnormal              ║"); anyAlert = true; }
  if (!anyAlert) { Serial.println("║  ✅ No active alerts            ║"); }
  
  Serial.println("╚═══════════════════════════════════╝\n");
}

// ===================================================================
// COMMUNICATION FUNCTIONS (MQTT)
// ===================================================================
void sendSensorData(bool forceImmediate) {
  if (!mqttClient.connected() || !systemEnabled) return;
  
  unsigned long currentTime = millis();
  if (!forceImmediate && (currentTime - lastDataSendTime < dataSendInterval)) {
    return;
  }
  
  jsonDoc.clear();
  
  JsonArray waterArray = jsonDoc.createNestedArray("water");
  waterArray.add(analogRead(WATER1));
  waterArray.add(analogRead(WATER2));
  waterArray.add(analogRead(WATER3));
  waterArray.add(analogRead(WATER4));
  
  jsonDoc["gas"] = (digitalRead(GAS_DO) == LOW);
  
  JsonObject tempObj = jsonDoc.createNestedObject("temperature");
  tempObj["temp1"] = readTemp(TEMP1);
  tempObj["temp2"] = readTemp(TEMP2);
  
  JsonObject gyroObj = jsonDoc.createNestedObject("gyro");
  mpu.update();
  float gyroMovement = abs(mpu.getGyroX()) + abs(mpu.getGyroY()) + abs(mpu.getGyroZ());
  gyroObj["movement"] = gyroMovement;
  gyroObj["x"] = mpu.getGyroX();
  gyroObj["y"] = mpu.getGyroY();
  gyroObj["z"] = mpu.getGyroZ();
  
  if (enableVoltageSensors || enableCurrentSensors) {
    JsonObject powerObj = jsonDoc.createNestedObject("power_readings");
    
    if (enableVoltageSensors) {
      powerObj["voltage1"] = convertVoltage(analogRead(VOLTAGE_SENSOR_1));
      powerObj["voltage2"] = convertVoltage(analogRead(VOLTAGE_SENSOR_2));
    }
    
    if (enableCurrentSensors) {
      powerObj["current1"] = convertCurrent(analogRead(CURRENT_SENSOR_1));
      powerObj["current2"] = convertCurrent(analogRead(CURRENT_SENSOR_2));
    }
  }
  
  JsonObject breakerObj = jsonDoc.createNestedObject("breakers");
  breakerObj["breaker1"] = breaker1State;
  breakerObj["breaker2"] = breaker2State;
  
  jsonDoc["system_enabled"] = systemEnabled;
  jsonDoc["buzzer_enabled"] = buzzerEnabled;
  jsonDoc["uptime"] = millis() / 1000;
  
  String output;
  serializeJson(jsonDoc, output);
  publishMessage(output.c_str());
  
  lastDataSendTime = currentTime;
}

void printHelp() {
  Serial.println("\n╔═══════════════════════════════════════════════════════════╗");
  Serial.println("║              AVAILABLE SERIAL COMMANDS                    ║");
  Serial.println("╠═══════════════════════════════════════════════════════════╣");
  Serial.println("║                                                           ║");
  Serial.println("║  SYSTEM COMMANDS:                                         ║");
  Serial.println("║    help       - Show this help menu                       ║");
  Serial.println("║    status     - Display full system status                ║");
  Serial.println("║    sensors    - Show current sensor readings              ║");
  Serial.println("║                                                           ║");
  Serial.println("║  WIFI COMMANDS:                                           ║");
  Serial.println("║    wifi       - Show WiFi connection info                 ║");
  Serial.println("║    scan       - Scan for available WiFi networks          ║");
  Serial.println("║                                                           ║");
  Serial.println("║  BREAKER COMMANDS:                                        ║");
  Serial.println("║    b1off      - Trip Breaker 1 OFF                        ║");
  Serial.println("║    b2off      - Trip Breaker 2 OFF                        ║");
  Serial.println("║    tripall    - Emergency trip all breakers               ║");
  Serial.println("║                                                           ║");
  Serial.println("║    ℹ️  Note: User must manually flip physical breakers   ║");
  Serial.println("║       back to ON position after tripping                  ║");
  Serial.println("║                                                           ║");
  Serial.println("║  TEST COMMANDS:                                           ║");
  Serial.println("║    shake      - Run 30-second gyro servo shake test       ║");
  Serial.println("║                                                           ║");
  Serial.println("╚═══════════════════════════════════════════════════════════╝");
  Serial.println("\n💡 TIP: Commands are case-insensitive\n");
}

void sendAlert(String alertType, String details) {
  if (!mqttClient.connected()) return;
  
  jsonDoc.clear();
  jsonDoc["alert"] = alertType;
  
  if (alertType == "WATER_DETECTED") {
    int w1 = analogRead(WATER1);
    int w2 = analogRead(WATER2);
    int w3 = analogRead(WATER3);
    int w4 = analogRead(WATER4);
    
    if (w1 > waterThreshold) {
      jsonDoc["sensor"] = "WATER1";
      jsonDoc["value"] = w1;
    } else if (w2 > waterThreshold) {
      jsonDoc["sensor"] = "WATER2";
      jsonDoc["value"] = w2;
    } else if (w3 > waterThreshold) {
      jsonDoc["sensor"] = "WATER3";
      jsonDoc["value"] = w3;
    } else if (w4 > waterThreshold) {
      jsonDoc["sensor"] = "WATER4";
      jsonDoc["value"] = w4;
    }
    jsonDoc["water1"] = w1;
    jsonDoc["water2"] = w2;
    jsonDoc["water3"] = w3;
    jsonDoc["water4"] = w4;
    
  } else if (alertType == "GAS_LEAK_DETECTED") {
    jsonDoc["sensor"] = "GAS";
    jsonDoc["value"] = 1;
    
  } else if (alertType == "HIGH_TEMPERATURE") {
    float t1 = readTemp(TEMP1);
    float t2 = readTemp(TEMP2);
    jsonDoc["value"] = max(t1, t2);
    jsonDoc["temp1"] = t1;
    jsonDoc["temp2"] = t2;
    if (t1 >= t2) {
      jsonDoc["sensor"] = "TEMP1";
    } else {
      jsonDoc["sensor"] = "TEMP2";
    }
    
  } else if (alertType == "GROUND_MOVEMENT_DETECTED") {
    mpu.update();
    float intensity = abs(mpu.getGyroX()) + abs(mpu.getGyroY()) + abs(mpu.getGyroZ());
    jsonDoc["intensity"] = intensity;
    jsonDoc["value"] = intensity;
    jsonDoc["sensor"] = "GYRO";
    jsonDoc["gyro_x"] = mpu.getGyroX();
    jsonDoc["gyro_y"] = mpu.getGyroY();
    jsonDoc["gyro_z"] = mpu.getGyroZ();
    
  } else if (alertType == "POWER_ABNORMAL") {
    jsonDoc["sensor"] = "POWER";
    if (enableVoltageSensors) {
      float v1 = convertVoltage(analogRead(VOLTAGE_SENSOR_1));
      float v2 = convertVoltage(analogRead(VOLTAGE_SENSOR_2));
      jsonDoc["voltage1"] = v1;
      jsonDoc["voltage2"] = v2;
      jsonDoc["value"] = max(v1, v2);
    }
    if (enableCurrentSensors) {
      float c1 = convertCurrent(analogRead(CURRENT_SENSOR_1));
      float c2 = convertCurrent(analogRead(CURRENT_SENSOR_2));
      jsonDoc["current1"] = c1;
      jsonDoc["current2"] = c2;
      if (!enableVoltageSensors) {
        jsonDoc["value"] = max(c1, c2);
      }
    }
  }
  
  if (details.length() > 0) {
    jsonDoc["details"] = details;
  }
  
  jsonDoc["timestamp"] = millis() / 1000;
  
  String output;
  serializeJson(jsonDoc, output);
  publishMessage(output.c_str());
}

void sendAlertCleared(String alertType) {
  if (!mqttClient.connected()) return;
  
  jsonDoc.clear();
  jsonDoc["status"] = "ALERT_CLEARED";
  jsonDoc["type"] = alertType;
  jsonDoc["timestamp"] = millis() / 1000;
  
  String output;
  serializeJson(jsonDoc, output);
  publishMessage(output.c_str());
}

void sendSystemStatus() {
  if (!mqttClient.connected()) return;
  
  jsonDoc.clear();
  jsonDoc["message_type"] = "SYSTEM_STATUS";
  jsonDoc["device_id"] = deviceId;
  jsonDoc["system_enabled"] = systemEnabled;
  jsonDoc["buzzer_enabled"] = buzzerEnabled;
  
  JsonObject thresh = jsonDoc.createNestedObject("thresholds");
  thresh["water"] = waterThreshold;
  thresh["temperature"] = tempThreshold;
  thresh["gyro"] = gyroThreshold;
  thresh["voltage"] = voltageThreshold;
  thresh["current"] = currentThreshold;
  
  JsonObject sensors = jsonDoc.createNestedObject("sensors");
  sensors["voltage_enabled"] = enableVoltageSensors;
  sensors["current_enabled"] = enableCurrentSensors;
  
  JsonObject breakers = jsonDoc.createNestedObject("breakers");
  breakers["breaker1"] = breaker1State;
  breakers["breaker2"] = breaker2State;
  
  jsonDoc["uptime"] = millis() / 1000;
  jsonDoc["firmware_version"] = "3.2-FIXED";
  
  String output;
  serializeJson(jsonDoc, output);
  publishMessage(output.c_str());
}

void sendAckResponse(String message) {
  if (!mqttClient.connected()) return;
  
  jsonDoc.clear();
  jsonDoc["status"] = "OK";
  jsonDoc["message"] = message;
  jsonDoc["device_id"] = deviceId;
  
  String output;
  serializeJson(jsonDoc, output);
  publishMessage(output.c_str());
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(GAS_DO, INPUT);
  pinMode(BUZZER, OUTPUT);
  
  initServos();

  Wire.begin(21, 22);
  mpu.begin();
  Serial.println("Calibrating gyro...");
  mpu.calcGyroOffsets(true);

  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║  ESP32 SAFETY MONITORING SYSTEM   ║");
  Serial.println("║     Version 3.2 (FIXED)           ║");
  Serial.println("╚═══════════════════════════════════╝\n");
  
  Serial.print("🌐 Connecting to: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(1000);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    prevWifiConnected = true;
    Serial.println("\n╔═══════════════════════════════════╗");
    Serial.println("║      ✅ WiFi CONNECTED!           ║");
    Serial.println("╠═══════════════════════════════════╣");
    Serial.print("║  IP: ");
    Serial.print(WiFi.localIP());
    Serial.println("            ║");
    Serial.print("║  Signal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm               ║");
    Serial.println("╚═══════════════════════════════════╝\n");
    
    deviceId = getDeviceId();
    telemetryTopic = "apn/device/" + deviceId + "/telemetry";
    commandTopic = "apn/device/" + deviceId + "/commands";
    
    Serial.println("📱 Device Configuration:");
    Serial.print("   Device ID: ");
    Serial.println(deviceId);
    Serial.print("   Telemetry Topic: ");
    Serial.println(telemetryTopic);
    Serial.print("   Command Topic: ");
    Serial.println(commandTopic);
    Serial.println();
    
    espClient.setCACert(root_ca);
    mqttClient.setServer(mqtt_broker, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(1024);
    
    mqttReconnect();
    
  } else {
    Serial.println("\n❌ WiFi FAILED - Check SSID/password");
    Serial.println("   ⚠️  Remember: ESP32 only works with 2.4GHz WiFi!");
    Serial.println("   Type 'scan' to see available networks\n");
  }
  
  Serial.println("✅ System Ready - Type 'help' for commands");
  Serial.println("─────────────────────────────────────────\n");
  
  prevBreaker1State = breaker1State;
  prevBreaker2State = breaker2State;
  prevSystemEnabled = systemEnabled;
  prevBuzzerEnabled = buzzerEnabled;
  
  digitalWrite(BUZZER, HIGH);
  delay(200);
  digitalWrite(BUZZER, LOW);
  
  lastHeartbeat = millis();
}

// ===================================================================
// MAIN LOOP
// ===================================================================
void loop() {
  unsigned long currentTime = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      if (currentTime - lastMqttReconnectAttempt > mqttReconnectInterval) {
        lastMqttReconnectAttempt = currentTime;
        if (mqttReconnect()) {
          lastMqttReconnectAttempt = 0;
        }
      }
    } else {
      mqttClient.loop();
    }
  }
  
  if (currentTime - lastWiFiCheck > 10000) {
    lastWiFiCheck = currentTime;
    
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    
    if (wifiConnected != prevWifiConnected) {
      if (wifiConnected) {
        Serial.println("\n╔═══════════════════════════════════╗");
        Serial.println("║      ✅ WiFi CONNECTED!           ║");
        Serial.println("╠═══════════════════════════════════╣");
        Serial.print("║  IP: ");
        Serial.print(WiFi.localIP());
        Serial.println("            ║");
        Serial.print("║  Signal: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm               ║");
        Serial.println("╚═══════════════════════════════════╝\n");
        
        wifiConnecting = false;
      } else {
        Serial.println("\n⚠️  WiFi DISCONNECTED!\n");
      }
      prevWifiConnected = wifiConnected;
    }
    
    if (!wifiConnected && !wifiConnecting) {
      wifiConnecting = true;
      WiFi.disconnect(true);
      delay(500);
      WiFi.begin(ssid, password);
    }
  }
  
  bool currentMqttConnected = mqttClient.connected();
  if (currentMqttConnected != prevMqttConnected) {
    if (currentMqttConnected) {
      Serial.println("📡 MQTT CONNECTED to cloud");
    } else {
      Serial.println("📡 MQTT DISCONNECTED");
    }
    prevMqttConnected = currentMqttConnected;
  }

 // SERIAL COMMANDS
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "shake") {
      testGyroShake();
    } else if (command == "b1off") {
      tripBreaker(1);
    } else if (command == "b2off") {
      tripBreaker(2);
    } else if (command == "tripall") {
      tripAllBreakers();
    } else if (command == "status") {
      printFullStatus();
    } else if (command == "wifi") {
      printWiFiInfo();
    } else if (command == "scan") {
      scanNetworks();
    } else if (command == "sensors") {
      printSensorReadings();
    } else if (command == "help") {
      printHelp();
    }
  }


  // Only run monitoring if system is enabled
  if (!systemEnabled) {
    if (prevSystemEnabled) {
      Serial.println("⏸️  System DISABLED");
      prevSystemEnabled = false;
    }
    delay(100);
    return;
  } else if (!prevSystemEnabled) {
    Serial.println("▶️  System ENABLED");
    prevSystemEnabled = true;
  }

  // ==================== CALL THE NEW ALERT HANDLER ====================
  handleAlerts();

  // ==================== HEARTBEAT ====================
  if (heartbeatInterval > 0 && (currentTime - lastHeartbeat >= heartbeatInterval)) {
    bool anyAlert = waterAlertActive || gasAlertActive || tempAlertActive || 
                    gyroAlertActive || powerAlertActive;
    
    if (!anyAlert) {
      Serial.printf("💚 System OK | Uptime: %lus | WiFi: %s | MQTT: %s\n",
                    currentTime / 1000,
                    (WiFi.status() == WL_CONNECTED) ? "✅" : "❌",
                    mqttClient.connected() ? "✅" : "⏳");
    }
    lastHeartbeat = currentTime;
  }

  // Optional periodic sensor data updates
  if (enablePeriodicUpdates && periodicUpdateInterval > 0) {
    if (currentTime - lastPeriodicUpdate >= periodicUpdateInterval) {
      sendSensorData(true);
      lastPeriodicUpdate = currentTime;
    }
  }

  delay(100);
}