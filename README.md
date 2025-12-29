# LIFTRR

ESP32 firmware for a lift-tracking device using a VL53L1X distance sensor and a BNO055 IMU. It renders an OLED UI, logs sessions to SD, supports BLE control, and streams session files over Bluetooth Classic. The codebase uses dependency injection to keep hardware access and app logic loosely coupled.

## Features
- Relative distance + orientation (roll/pitch/yaw)
- OLED UI with status, tracking, and orientation warning screen
- SD session logging (CSV + index)
- BLE control protocol (modes, sessions, time sync, listing, file request)
- Bluetooth Classic file streaming for sessions
- Auto-IDLE after 30s without motion

## Hardware
- ESP32 dev board (Arduino framework)
- BNO055 IMU (I2C)
- VL53L1X ToF distance sensor (I2C)
- SSD1306 128x64 OLED (I2C, address 0x3C)
- microSD card module (SPI)
- Optional SD activity LED on `LED_SD`

## Pins / bus
- I2C: SDA 21, SCL 22 (set in `src/core/main.cpp`)
- OLED address: 0x3C (set in `src/core/config.h`)
- SD CS: 13 (`SD_CS` in `src/core/config.h`)
- SD activity LED: 2 (`LED_SD` in `src/core/config.h`)

## Build and upload (PlatformIO)
```
pio run
pio run -t upload
pio device monitor -b 115200
```
Set `upload_port` / `monitor_port` in `platformio.ini` if needed.

## Runtime modes and UI
- RUN: live sensing; logging only while a session is active
- CALIBRATE: shown when IMU/laser not ready
- IDLE: entered after 30s of no motion
- DUMP: dedicated screen for dump/debug

If the device is facing LEFT/RIGHT, the tracking screen is replaced by an orientation warning screen.

## Architecture overview
- Composition root in `src/core/main.cpp` instantiates hardware (I2C, OLED, SD, sensors) and injects them into subsystem managers.
- `liftrr::sensors::SensorManager` owns calibration state, offsets, and provides pose samples.
- `liftrr::storage::StorageManager` owns SD session logging and the session index.
- `liftrr::app::DisplayManager` renders screens using injected UI, BLE, storage, and sensor state.
- `liftrr::ble::BleApp` owns BLE app behavior and delegates to `liftrr::ble::BleManager`.
- `liftrr::core::RuntimeState` owns device mode and runtime timers (no globals).

## Serial commands
From `src/app/serial_commands.cpp`:
- `m`: cycle RUN -> DUMP -> IDLE
- `r`: force RUN
- `s`: start session (only if RUN and no active session)
- `e`: end session
- `i`: print session index and directory info
- `d`: print calibration + distance debug

## BLE control
Device name: `LIFTRR` (MTU 185)

Commands (JSON over BLE):
```
{"id":"1","name":"ping","body":{}}
{"id":"2","name":"mode.set","body":{"mode":"RUN"}}
{"id":"3","name":"session.start","body":{"lift":"deadlift","sessionId":"optional"}}
{"id":"4","name":"session.end","body":{}}
{"id":"5","name":"sessions.list","body":{"cursor":0,"limit":15}}
{"id":"6","name":"time.sync","body":{"phoneEpochMs":1710000000000}}
{"id":"7","name":"session.stream","body":{"sessionId":"1710000000000"}}
```

Notes:
- On BLE connect, the device emits `time.sync.request` and times out after 10s if no reply.
- `session.start` returns `CALIBRATION_REQUIRED` until IMU + laser are ready; it auto-starts when ready.
- `session.stream` requests a file transfer over Bluetooth Classic (see below).

Events:
- `orientation.status` with `{facing, ok}`
- `calibration.succeeded` with `{imu, laser, ready}`
- `bt_classic.required` when Classic is not connected on BLE connect

## Bluetooth Classic file streaming
When the phone sends `session.stream` over BLE, the device checks the SD index and streams the file over Classic Bluetooth if connected.

Classic stream format:
- JSON line: `{"event":"session.file.start","sessionId":"...","size":1234}`
- Raw file bytes (CSV)
- JSON line: `{"event":"session.file.end","sessionId":"...","size":1234}`

## Data logging format
- Active session file: `/sessions/<sessionId>.tmp`
- Finalized session file: `/sessions/<sessionId>.csv`
- Index: `/sessions/index.ndjson`

Session files include comment headers, then CSV rows:
```
# liftrr session
# session_id=...
# exercise=...
# calib_laserOffset=...
# calib_rollOffset=...
# calib_pitchOffset=...
# calib_yawOffset=...
timestamp_ms,dist_mm,relDist_mm,roll_deg,pitch_deg,yaw_deg
```

Index lines:
```
{"name":"<file>","size":1234,"mtime":0}
```

## Repo layout
- `src/core/`: main loop, runtime state, config, RTC
- `src/sensors/`: sensor interfaces, adapters, and sensor manager
- `src/storage/`: SD logging manager and index helpers
- `src/ui/`: OLED drawing helpers (UI renderer)
- `src/comm/`: Bluetooth Classic
- `src/ble/`: BLE protocol, manager, and app wrapper
- `src/app/`: display manager, motion controller, serial commands
- `lib/`, `include/`, `test/`: PlatformIO standard structure

Dependencies are listed in `platformio.ini`.
