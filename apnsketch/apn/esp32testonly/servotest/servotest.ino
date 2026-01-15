/*
 * ========================================
 * SERVO MOTOR TEST - FAST & WIDE EARTHQUAKE
 * ========================================
 * Pins:
 *   Servo 1 (Earthquake): GPIO 16
 *   Servo 2 (Breaker):    GPIO 18
 * ========================================
 */

#include <ESP32Servo.h>

// Check your wiring! 
// Standard ESP32 pins often used: 16, 17, 18, 19, 21, 22, 23
#define SERVO_EARTHQUAKE  16 
#define SERVO_BREAKER     18 

Servo servoEarthquake;
Servo servoBreaker;
        
int posEarthquake = 90;
int posBreaker = 90;
int selectedServo = 1;

// --- CONSTANTS ---
const int BREAKER_ON = 90;
const int BREAKER_OFF = 180; 

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  // Wide calibration for max movement
  servoEarthquake.attach(SERVO_EARTHQUAKE, 500, 2500);
  servoBreaker.attach(SERVO_BREAKER, 500, 2500);
  
  servoEarthquake.write(90);
  servoBreaker.write(BREAKER_ON);
  
  Serial.println("=== FAST EARTHQUAKE MODE ===");
  Serial.println("Commands: e (quake), t (trip), r (reset)");
}

void earthquakeSimulation() {
  Serial.println(">>> EARTHQUAKE: FAST & WIDE <<<");
  
  // 1. FAST JITTER (P-Wave simulation)
  // Very fast small movements
  for (int cycle = 0; cycle < 12; cycle++) {
    int shake1 = random(60, 90);
    int shake2 = random(90, 120);
    
    servoEarthquake.write(shake1);
    delay(30); // Reduced from 60 to 30 for speed
    servoEarthquake.write(shake2);
    delay(30);
  }
  
  // 2. VIOLENT MAIN SHOCK (S-Wave simulation)
  // Maximum Width + High Speed
  for (int cycle = 0; cycle < 15; cycle++) {
    // Swing LEFT (almost 0)
    servoEarthquake.write(5);   
    delay(45); // Reduced from 100 to 45. 
               // This is very fast. The servo will try its hardest.
    
    // Swing RIGHT (almost 180)
    servoEarthquake.write(175); 
    delay(45); 
  }
  
  // 3. RAPID DECAY
  for (int cycle = 0; cycle < 8; cycle++) {
    int range = 50 - (cycle * 6); 
    if(range < 5) range = 5;

    servoEarthquake.write(90 - range);
    delay(50 + cycle * 5); // Keep decay slightly faster too
    servoEarthquake.write(90 + range);
    delay(50 + cycle * 5);
  }
  
  servoEarthquake.write(90);
  posEarthquake = 90;
  Serial.println("Simulation End.");
}

void tripBreaker() {
  Serial.println("Tripping Breaker...");
  for (int i = BREAKER_ON; i <= BREAKER_OFF; i += 8) { // Increased speed (+8)
    servoBreaker.write(i);
    delay(10);
  }
  servoBreaker.write(BREAKER_OFF);
  posBreaker = BREAKER_OFF;
}

void resetBreaker() {
  Serial.println("Resetting Breaker...");
  for (int i = BREAKER_OFF; i >= BREAKER_ON; i -= 5) { // Increased speed (-5)
    servoBreaker.write(i);
    delay(15);
  }
  servoBreaker.write(BREAKER_ON);
  posBreaker = BREAKER_ON;
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'e') earthquakeSimulation();
    if (c == 't') tripBreaker();
    if (c == 'r') resetBreaker();
    if (c == 'c') {
       servoEarthquake.write(90);
       servoBreaker.write(90);
    }
  }
  delay(10);
}