/*
 * ========================================
 * BUZZER TEST (2-Pin Buzzer)
 * ========================================
 * Wiring:
 *   Pin 1 (+) → GPIO 19
 *   Pin 2 (-) → GND
 * ========================================
 */

#define BUZZER_PIN 19

// Musical note frequencies
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

void beep(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println("================================");
  Serial.println("       BUZZER TEST (2-Pin)");
  Serial.println("================================");
  Serial.println("Wiring:");
  Serial.println("  Buzzer (+) --> GPIO 19");
  Serial.println("  Buzzer (-) --> GND");
  Serial.println("================================");
  Serial.println("Commands:");
  Serial.println("  1 - Short beep");
  Serial.println("  2 - Long beep");
  Serial.println("  3 - Double beep");
  Serial.println("  4 - Warning alarm");
  Serial.println("  5 - Success melody");
  Serial.println("  6 - Error sound");
  Serial.println("  7 - Musical scale");
  Serial.println("  8 - Siren");
  Serial.println("  9 - Earthquake alarm");
  Serial.println("  0 - Stop buzzer");
  Serial.println("================================");
  
  // Startup beep
  beep(1000, 100);
  delay(100);
  beep(1500, 100);
  
  Serial.println("Ready! Send a command.");
}

void shortBeep() {
  Serial.println("Short beep");
  beep(1000, 100);
}

void longBeep() {
  Serial.println("Long beep");
  beep(1000, 500);
}

void doubleBeep() {
  Serial.println("Double beep");
  beep(1000, 100);
  delay(100);
  beep(1000, 100);
}

void warningAlarm() {
  Serial.println("Warning alarm");
  for (int i = 0; i < 3; i++) {
    beep(2000, 150);
    delay(100);
    beep(1500, 150);
    delay(100);
  }
}

void successMelody() {
  Serial.println("Success melody");
  beep(NOTE_C4, 100);
  delay(50);
  beep(NOTE_E4, 100);
  delay(50);
  beep(NOTE_G4, 100);
  delay(50);
  beep(NOTE_C5, 200);
}

void errorSound() {
  Serial.println("Error sound");
  beep(400, 200);
  delay(100);
  beep(300, 300);
}

void playScale() {
  Serial.println("Musical scale");
  int notes[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, 
                 NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};
  
  for (int i = 0; i < 8; i++) {
    beep(notes[i], 150);
    delay(50);
  }
}

void siren() {
  Serial.println("Siren");
  for (int j = 0; j < 2; j++) {
    for (int i = 500; i <= 1500; i += 30) {
      tone(BUZZER_PIN, i);
      delay(10);
    }
    for (int i = 1500; i >= 500; i -= 30) {
      tone(BUZZER_PIN, i);
      delay(10);
    }
  }
  noTone(BUZZER_PIN);
}

void earthquakeAlarm() {
  Serial.println("EARTHQUAKE ALARM!");
  for (int i = 0; i < 5; i++) {
    beep(2500, 100);
    beep(1500, 100);
  }
}

void stopBuzzer() {
  noTone(BUZZER_PIN);
  Serial.println("Buzzer stopped");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '1': shortBeep(); break;
      case '2': longBeep(); break;
      case '3': doubleBeep(); break;
      case '4': warningAlarm(); break;
      case '5': successMelody(); break;
      case '6': errorSound(); break;
      case '7': playScale(); break;
      case '8': siren(); break;
      case '9': earthquakeAlarm(); break;
      case '0': stopBuzzer(); break;
    }
  }
  delay(10);
}