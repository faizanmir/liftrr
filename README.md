# LIFTRR

ESP32 firmware for a lift-tracking device that combines a VL53L1X laser range
sensor and a BNO055 IMU, shows status on an SSD1306 OLED, logs sessions to SD,
and exposes BLE control.

## Features
- Real-time relative distance and orientation (roll/pitch/yaw)
- OLED UI with status, motion bar, and horizon indicator
- SD card session logging (CSV + index)
- BLE control protocol (mode set, session start/end, time sync, session list)
- Auto IDLE after 30s without motion

## Hardware
- ESP32 dev board (Arduino framework)
- BNO055 IMU (I2C)
- VL53L1X time-of-flight distance sensor (I2C)
- SSD1306 128x64 OLED (I2C, address 0x3C)
- microSD card module (SPI)
- Optional: LED on `LED_SD` for write pulses

## Pins / bus
- I2C: SDA 21, SCL 22 (set in `src/main.cpp`)
- OLED address: 0x3C (set in `src/config.h`)
- SD CS: 13 (`SD_CS` in `src/config.h`)
- SD activity LED: 2 (`LED_SD` in `src/config.h`)

## Build and upload (PlatformIO)
```
pio run
pio run -t upload
pio device monitor -b 115200
```
If needed, set `upload_port` / `monitor_port` in `platformio.ini`.

## Runtime modes and UI
- RUN: live sensing; logging only while a session is active
- CALIBRATE: shown when IMU/laser not ready; displays calibration hints
- IDLE: entered after 30s of no motion
- DUMP: dedicated screen for dump/debug

UI note: The SSD1306 is monochrome; the UI uses inverse blocks, outlines, and
striped fills to visually separate sections.

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

The phone sends JSON commands like:
```
{"id":"1","name":"ping","body":{}}
{"id":"2","name":"mode.set","body":{"mode":"RUN"}}
{"id":"3","name":"session.start","body":{"lift":"deadlift","sessionId":"optional"}}
{"id":"4","name":"session.end","body":{}}
{"id":"5","name":"sessions.list","body":{"cursor":0,"limit":15}}
{"id":"6","name":"time.sync","body":{"phoneEpochMs":1710000000000}}
```

Supported commands:
- `ping`
- `capabilities.get`
- `time.sync`
- `mode.set`
- `session.start`
- `session.end`
- `sessions.list`

Notes:
- `session.start` returns `CALIBRATION_REQUIRED` if the IMU/laser are not ready.
  The device will auto-start once ready and emit `evt: session.started`.

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

Index lines look like:
```
{"name":"<file>","size":1234,"mtime":0}
```

## Repo layout
- `src/`: firmware source
- `src/app/`: motion logic, display rendering, serial commands
- `src/ble/`: BLE manager and app protocol
- `src/ui.*`: OLED drawing helpers
- `lib/`, `include/`, `test/`: PlatformIO standard structure

Dependencies are declared in `platformio.ini`.
