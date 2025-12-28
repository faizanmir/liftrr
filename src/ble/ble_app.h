#pragma once

namespace liftrr {
namespace ble {

// Call once from setup()
void bleAppInit();

// Call every loop() iteration (will internally call BleManager.loop())
void bleAppLoop();

// Interface: lets the application decide what a mode change means
// (e.g., set deviceMode, reset inactivity timers, wake display, etc.).
struct IModeApplier {
  virtual ~IModeApplier() = default;
  // mode is expected to be one of: "RUN", "IDLE", "DUMP".
  virtual void applyMode(const char* mode) = 0;
};

// Register an optional mode applier. If not set, ble_app.cpp will fall back
// to its current internal behavior.
void bleAppSetModeApplier(IModeApplier* applier);
} // namespace ble
} // namespace liftrr
