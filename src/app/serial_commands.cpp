#include "serial_commands.h"

#include "globals.h"
#include "storage.h"
#include "sensors.h"
#include "rtc.h"
#include <SD.h>

namespace liftrr {
namespace app {

void handleSerialCommands(MotionState &motionState) {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
        case 'm':
            // Cycle device mode manually
            deviceMode = static_cast<DeviceMode>(
                    (static_cast<int>(deviceMode) + 1) % 3);
            Serial.print("Device Mode changed to: ");
            if (deviceMode == MODE_RUN)  Serial.println("MODE_RUN");
            if (deviceMode == MODE_DUMP) Serial.println("MODE_DUMP");
            if (deviceMode == MODE_IDLE) Serial.println("MODE_IDLE");
            motionState.lastMotionTime = millis(); // reset idle timer
            break;

        case 'r':
            deviceMode = MODE_RUN;
            motionState.lastMotionTime = millis();
            Serial.println("Forced MODE_RUN");
            break;

        case 's': {
            // Start a session with a simple auto-generated ID
            int64_t epoch = liftrr::currentEpochMs();
            String sid = (epoch > 0) ? String((long long)epoch) : String(millis());
            if (storageIsSessionActive()) {
                Serial.println("Session already active, cannot start new one.");
                break;
            }
            if (deviceMode != MODE_RUN) {
                Serial.println("Device not in RUN mode, cannot start session.");
                break;
            }
            if (deviceMode == MODE_RUN) {
                storageStartSession(sid,
                                    "lift",
                                    laserOffset,
                                    rollOffset,
                                    pitchOffset,
                                    yawOffset);
                Serial.println("Session started via serial 's'");
            }
            break;
        }

        case 'e':
            if (!storageIsSessionActive()) {
                Serial.println("No active session to end.");
                break;
            }

            storageEndSession();
            Serial.println("Session ended via serial 'e'");
            break;

        case 'i': {
            if (!storageInitSd()) {
                Serial.println("SD init failed.");
                break;
            }

            Serial.println("--- /sessions/index.ndjson ---");
            File idx = SD.open("/sessions/index.ndjson", FILE_READ);
            if (idx) {
                while (idx.available()) {
                    String line = idx.readStringUntil('\n');
                    line.trim();
                    if (line.length()) Serial.println(line);
                }
                idx.close();
            } else {
                Serial.println("(missing)");
            }

            Serial.println("--- /sessions entries ---");
            File dir = SD.open("/sessions");
            if (!dir || !dir.isDirectory()) {
                Serial.println("(missing or not a dir)");
                if (dir) dir.close();
                break;
            }

            File entry = dir.openNextFile();
            while (entry) {
                Serial.print(entry.isDirectory() ? "DIR  " : "FILE ");
                Serial.print(entry.name());
                Serial.print("  ");
                Serial.println(entry.size());
                entry.close();
                entry = dir.openNextFile();
            }
            dir.close();
            break;
        }

        case 'd': {
            SensorSample sample;
            sensorsRead(sample);
            Serial.print("cal: s="); Serial.print(sample.s);
            Serial.print(" g="); Serial.print(sample.g);
            Serial.print(" a="); Serial.print(sample.a);
            Serial.print(" m="); Serial.println(sample.m);
            Serial.print("laserValid="); Serial.print(laserValid ? "1" : "0");
            Serial.print(" dist="); Serial.println(sample.rawDist);
            Serial.print("isCalibrated="); Serial.println(isCalibrated ? "1" : "0");
            break;
        }

        default:
            // ignore unknown commands
            break;
    }
}

} // namespace app
} // namespace liftrr
