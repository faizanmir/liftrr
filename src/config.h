#pragma once

// --- HARDWARE CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C 

// Pins
#define TARE_BTN_PIN 4  
#define FLASH_CS     5
#define FLASH_SCK    18
#define FLASH_MISO   19
#define FLASH_MOSI   23
#define SD_CS 13
#define LED_SD     2

// Timing
const long SCREEN_INTERVAL = 100;    // 10Hz Screen Update
const long LOG_INTERVAL = 50;        // 20Hz Data Logging
const long AUTO_DUMP_INTERVAL = 100000; // 100s Auto-Dump Timer

enum DeviceMode {
  MODE_RUN,   // sensing + logging
  MODE_DUMP ,
  MODE_IDLE
};