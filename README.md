# LIFTRR

ESP32 firmware for a lift-tracking device using a VL53L1X ToF distance sensor and a BNO055 IMU. It renders an SSD1306 OLED UI, logs sessions to SD, exposes a BLE JSON control protocol, and streams session files over Bluetooth Classic.

## Features
- Relative distance + orientation (roll/pitch/yaw)
- OLED UI with status, tracking, calibration, idle, dump, and orientation warning screens
- SD session logging (CSV + NDJSON index)
- BLE JSON control protocol (time sync, modes, sessions, listing, file request)
- Bluetooth Classic file streaming for sessions
- Auto-IDLE after 30s without motion, wake on motion
- Serial debug commands (single-character + JSON)

## Hardware
- ESP32 dev board (Arduino framework)
- BNO055 IMU (I2C, 0x28)
- VL53L1X ToF distance sensor (I2C, 0x29)
- SSD1306 128x64 OLED (I2C, 0x3C)
- microSD card module (SPI)
- Optional SD activity LED on `LED_SD`

## Pins / buses
- I2C: SDA 21, SCL 22 (`Wire.begin(21, 22)` in `src/core/main.cpp`)
- OLED address: 0x3C (`SCREEN_ADDRESS` in `src/core/config.h`)
- VL53L1X address: 0x29 (set in `src/core/main.cpp`)
- SD CS: 13 (`SD_CS` in `src/core/config.h`)
- SD activity LED: 2 (`LED_SD` in `src/core/config.h`)
- `TARE_BTN_PIN` and `FLASH_*` are defined in `src/core/config.h` but not used in firmware.

## Build and upload (PlatformIO)
```
pio run
pio run -t upload
pio device monitor -b 115200
```
Set `upload_port` / `monitor_port` in `platformio.ini` if needed.

## Runtime modes and UI
- RUN: live sensing; logging only while a session is active
- CALIBRATE: shown when IMU or laser are not ready; auto-switches to RUN when ready
- IDLE: entered after 30s of no motion; any motion returns to RUN
- DUMP: dedicated screen; sensing/logging paused

If the device is facing LEFT/RIGHT, the tracking screen is replaced by an orientation warning screen.

## Serial control
Single-character commands:
- `m`: cycle RUN -> DUMP -> IDLE
- `r`: force RUN
- `s`: start session (auto-generated ID)
- `e`: end session
- `i`: print session index and directory info
- `d`: print calibration + distance debug

JSON commands (newline-terminated, one per line):
```
{"id":"1","name":"ping","body":{}}
{"id":"2","name":"capabilities.get","body":{}}
{"id":"3","name":"time.sync","body":{"phoneEpochMs":1710000000000}}
{"id":"4","name":"mode.set","body":{"mode":"RUN"}}
{"id":"5","name":"session.start","body":{"lift":"deadlift","sessionId":"optional"}}
{"id":"6","name":"session.end","body":{}}
{"id":"7","name":"sessions.list","body":{"cursor":0,"limit":15}}
{"id":"8","name":"session.stream","body":{"sessionId":"1710000000000"}}
```
Use "Newline" line ending in the serial monitor.

## BLE control
Device name: `LIFTRR` (MTU 185)

Commands (JSON over BLE):
```
{"id":"1","name":"ping","body":{}}
{"id":"2","name":"capabilities.get","body":{}}
{"id":"3","name":"mode.set","body":{"mode":"RUN"}}
{"id":"4","name":"session.start","body":{"lift":"deadlift","sessionId":"optional"}}
{"id":"5","name":"session.end","body":{}}
{"id":"6","name":"sessions.list","body":{"cursor":0,"limit":15}}
{"id":"7","name":"time.sync","body":{"phoneEpochMs":1710000000000}}
{"id":"8","name":"session.stream","body":{"sessionId":"1710000000000"}}
```

Notes:
- On BLE connect, the device emits `time.sync.request` and times out after 10s if no reply.
- `session.start` returns `CALIBRATION_REQUIRED` until IMU + laser are ready; it auto-starts when ready.
- `session.stream` requests a file transfer over Bluetooth Classic (see below).

Events:
- `orientation.status` with `{facing, ok}`
- `calibration.succeeded` with `{imu, laser, ready}`
- `session.started` when a pending session auto-starts
- `bt_classic.required` when Classic is not connected on BLE connect
- `time.sync.timeout` when the sync window expires

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
- `src/core/`: main loop, runtime state, config, time sync
- `src/sensors/`: sensor interfaces, adapters, and sensor manager
- `src/storage/`: SD logging manager and index helpers
- `src/ui/`: OLED drawing helpers
- `src/comm/`: Bluetooth Classic streaming
- `src/ble/`: BLE protocol, manager, and app wrapper
- `src/app/`: display manager, motion controller, serial commands
- `lib/`, `include/`, `test/`: PlatformIO standard structure

Dependencies are listed in `platformio.ini`.
