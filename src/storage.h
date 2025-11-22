#pragma once

#include <stdint.h>
#include <stddef.h>
#include "globals.h"

// Single sample of lift data
struct LiftSample {
  uint32_t t;     // time in ms
  int16_t  dist;  // distance in mm (relative)
  float    roll;  // degrees
  float    pitch; // degrees
  float    yaw;   // degrees
};

// --- Flash lifecycle ---
// Initialise flash and restore or create log header
bool initFlashStorage();
// Erase header + log region and reset write pointer
bool flashFactoryReset();

// --- Logging primitives ---
// Append raw bytes to the log region
bool flashAppend(const void* data, size_t len);
// Log a LiftSample as a CSV line: t,dist,roll,pitch,yaw
bool logLiftSampleCsv(const LiftSample& sample);
// Simple helper for logging a CSV line from current globals
bool writeDummySampleToFlash();

// --- Reading utilities ---
// Read back a raw chunk from the log region
bool readLogChunk(uint32_t addr, uint8_t* buf, size_t len);