#include "serial_commands.h"

#include "core/globals.h"
#include "storage/storage.h"
#include "sensors/sensors.h"
#include "core/rtc.h"
#include <SD.h>

namespace liftrr {
namespace app {

void handleSerialCommands(MotionState &motionState) {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
        case 'm':
            // Cycle device mode manually
            liftrr::core::deviceMode = static_cast<liftrr::core::DeviceMode>(
                    (static_cast<int>(liftrr::core::deviceMode) + 1) % 3);
            Serial.print("Device Mode changed to: ");
            if (liftrr::core::deviceMode == liftrr::core::MODE_RUN)  Serial.println("MODE_RUN");
            if (liftrr::core::deviceMode == liftrr::core::MODE_DUMP) Serial.println("MODE_DUMP");
            if (liftrr::core::deviceMode == liftrr::core::MODE_IDLE) Serial.println("MODE_IDLE");
            motionState.lastMotionTime = millis(); // reset idle timer
            break;

        case 'r':
            liftrr::core::deviceMode = liftrr::core::MODE_RUN;
            motionState.lastMotionTime = millis();
            Serial.println("Forced MODE_RUN");
            break;

        case 's': {
            // Start a session with a simple auto-generated ID
            int64_t epoch = liftrr::core::currentEpochMs();
            String sid = (epoch > 0) ? String((long long)epoch) : String(millis());
            if (liftrr::storage::storageIsSessionActive()) {
                Serial.println("Session already active, cannot start new one.");
                break;
            }
            if (liftrr::core::deviceMode != liftrr::core::MODE_RUN) {
                Serial.println("Device not in RUN mode, cannot start session.");
                break;
            }
            if (liftrr::core::deviceMode == liftrr::core::MODE_RUN) {
                liftrr::storage::storageStartSession(sid,
                                    "lift",
                                    liftrr::core::laserOffset,
                                    liftrr::core::rollOffset,
                                    liftrr::core::pitchOffset,
                                    liftrr::core::yawOffset);
                Serial.println("Session started via serial 's'");
            }
            break;
        }

        case 'e':
            if (!liftrr::storage::storageIsSessionActive()) {
                Serial.println("No active session to end.");
                break;
            }

            liftrr::storage::storageEndSession();
            Serial.println("Session ended via serial 'e'");
            break;

        case 'i': {
            if (!liftrr::storage::storageInitSd()) {
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
            liftrr::sensors::SensorSample sample;
            liftrr::sensors::sensorsRead(sample);
            Serial.print("cal: s="); Serial.print(sample.s);
            Serial.print(" g="); Serial.print(sample.g);
            Serial.print(" a="); Serial.print(sample.a);
            Serial.print(" m="); Serial.println(sample.m);
            Serial.print("laserValid="); Serial.print(liftrr::core::laserValid ? "1" : "0");
            Serial.print(" dist="); Serial.println(sample.rawDist);
            Serial.print("isCalibrated="); Serial.println(liftrr::core::isCalibrated ? "1" : "0");
            break;
        }

        default:
            // ignore unknown commands
            break;
    }
}

} // namespace app
} // namespace liftrr
