#include "globals.h"
#include "ui.h"
#include "storage.h"

void setUpFlash() {
    if (initFlashStorage()) {
        Serial.println("Flash storage initialized successfully.");
        display.setCursor(0, 40);
        display.println("Flash Init OK");
        display.display();
        delay(500);
    } else {
        Serial.println("Flash storage initialization failed.");
        display.setCursor(0, 40);
        display.println("Flash Init FAIL");
        display.display();
        delay(500);
    }
}



void setup() {
  Serial.begin(115200);
  while(!Serial); 
  Serial.println("--- SYSTEM START ---");

  pinMode(TARE_BTN_PIN, INPUT_PULLUP);

  // 1. Init I2C Busz
  Wire.begin(21, 22); 
  Wire.setClock(100000); 
  Serial.println("I2C Bus Initialized");

  // 2. Init Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display Init Failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("BOOT SEQUENCE...");
    display.display();
  }
    // 3. Init Flash Storage
    setUpFlash();

  // 4. Init Sensors
  if(!bno.begin()) { Serial.println("BNO Fail"); while(1); }
  bno.setExtCrystalUse(true);

  if (!laser.begin(0x29, &Wire)) { Serial.println("Laser Fail"); while(1); }
  laser.startRanging();
  laser.setTimingBudget(50);
  
  Serial.println("--- SETUP COMPLETE ---");
  delay(500);
}

void loop() {
  // --- Serial Commands ---
  if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'd') writeDummySampleToFlash();
      if (cmd == 'c') flashFactoryReset();
  }

  unsigned long currentMillis = millis();

  // --- 1. Read Sensors ---
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t s, g, a, m;
  bno.getCalibration(&s, &g, &a, &m);

  if (laser.dataReady()) {
    int16_t newDist = laser.distance();
    if (newDist != -1) { 
        distance = newDist;
        laserValid = true; 
    }
    laser.clearInterrupt();
  }

  // --- 2. Logic: Check Readiness ---
  bool bnoReady = (s >= 2); 
  if (bnoReady && laserValid) {
    isCalibrated = true;
  } else {
    isCalibrated = false;
  }

  // --- 3. Button Logic (Tare/Reset) ---
  int btnState = digitalRead(TARE_BTN_PIN);
  if (btnState == LOW && lastBtnState == HIGH) {
      btnPressTime = currentMillis;
  }
  if (btnState == HIGH && lastBtnState == LOW) {
      unsigned long duration = currentMillis - btnPressTime;
      if (duration > 2000) {
          // RESET (Long Press)
          laserOffset = 0; 
          rollOffset = 0; pitchOffset = 0; yawOffset = 0;
          display.clearDisplay();
          display.setCursor(30, 25); display.setTextSize(2); display.println("RESET");
          display.display(); delay(500);
      } else if (duration > 50 && isCalibrated) {
          // TARE (Short Press)
          rollOffset = event.orientation.y;
          pitchOffset = event.orientation.z;
          yawOffset = event.orientation.x; 
          laserOffset = distance;
          
          // Visual feedback only; logging handled via raw flash, no file init needed
          display.invertDisplay(true); delay(100); display.invertDisplay(false);
      }
  }
  lastBtnState = btnState;

  // --- 4. Update Display (10Hz) ---
  if (currentMillis - lastScreenUpdate >= SCREEN_INTERVAL) {
    lastScreenUpdate = currentMillis;
    display.clearDisplay();

    bool tracking = (laserOffset != 0);
    drawStatusBar(tracking);

    if (!isCalibrated || !laserValid) {
       // Calibration Screen
       display.setCursor(20, 30); display.setTextSize(1);
       if(!isCalibrated) display.println("Calibrating IMU...");
       else display.println("Laser Waiting...");
       
       display.setCursor(0, 50);
       display.print("G:"); display.print(g);
       display.print(" A:"); display.print(a);
       display.print(" M:"); display.print(m);
    } 
    else {
        // Tracking HUD
        int16_t relDist = distance - laserOffset;
        float relRoll = event.orientation.y - rollOffset;
        float relPitch = event.orientation.z - pitchOffset;
        float relYaw = event.orientation.x - yawOffset;
        if (relYaw > 180) relYaw -= 360;
        if (relYaw < -180) relYaw += 360;

        drawVerticalBar(relDist);
        drawHorizon(relRoll);

        // Dynamic Text Centering
        int xPos = 10; 
        if (abs(relDist) < 10) xPos = 45;
        else if (abs(relDist) < 100) xPos = 35;
        else if (abs(relDist) < 1000) xPos = 15;

        display.setTextSize(3);
        display.setCursor(xPos, 20);
        display.print(relDist);
        
        display.setTextSize(1);
        display.setCursor(xPos + (abs(relDist)>=1000?75:(abs(relDist)>=100?55:38)), 34);
        display.print("mm");

        display.setCursor(0, 48);
        display.print("P:"); display.print((int)relPitch);
        display.setCursor(60, 48);
        display.print("Y:"); display.print((int)relYaw);
    }
    display.display();
  }
  
  // --- 5. Logging (20Hz) ---
  bool tracking = (laserOffset != 0);
  if (tracking && flashReady && (currentMillis - lastLogTime >= 50)) {   // 20 Hz = 50 ms
      lastLogTime = currentMillis;

      // Build a binary sample for this moment
      LiftSample sample;
      sample.t    = currentMillis;
      sample.dist = distance - laserOffset;
      sample.roll = event.orientation.y - rollOffset;
      sample.pitch= event.orientation.z - pitchOffset;
      sample.yaw  = event.orientation.x - yawOffset;
      if (sample.yaw > 180) sample.yaw -= 360;
      if (sample.yaw < -180) sample.yaw += 360;

      if (!flashAppend(&sample, sizeof(sample))) {
          Serial.println("Failed to append LiftSample to flash");
      }
  }
}