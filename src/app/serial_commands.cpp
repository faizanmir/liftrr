#include "serial_commands.h"

#include "core/rtc.h"
#include <SD.h>

namespace liftrr {
namespace app {

SerialCommandHandler::SerialCommandHandler(liftrr::core::RuntimeState &runtime,
                                           liftrr::storage::StorageManager &storage,
                                           liftrr::sensors::SensorManager &sensors)
    : runtime_(runtime), storage_(storage), sensors_(sensors) {}

void SerialCommandHandler::handleSerialCommands(MotionState &motionState) {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
        case 'm':
            // Cycle device mode manually
            runtime_.setDeviceMode(static_cast<liftrr::core::DeviceMode>(
                    (static_cast<int>(runtime_.deviceMode()) + 1) % 3));
            Serial.print("Device Mode changed to: ");
            if (runtime_.deviceMode() == liftrr::core::MODE_RUN)  Serial.println("MODE_RUN");
            if (runtime_.deviceMode() == liftrr::core::MODE_DUMP) Serial.println("MODE_DUMP");
            if (runtime_.deviceMode() == liftrr::core::MODE_IDLE) Serial.println("MODE_IDLE");
            motionState.lastMotionTime = millis(); // reset idle timer
            break;

        case 'r':
            runtime_.setDeviceMode(liftrr::core::MODE_RUN);
            motionState.lastMotionTime = millis();
            Serial.println("Forced MODE_RUN");
            break;

        case 's': {
            // Start a session with a simple auto-generated ID
            int64_t epoch = liftrr::core::currentEpochMs();
            String sid = (epoch > 0) ? String((long long)epoch) : String(millis());
            if (storage_.isSessionActive()) {
                Serial.println("Session already active, cannot start new one.");
                break;
            }
            if (runtime_.deviceMode() != liftrr::core::MODE_RUN) {
                Serial.println("Device not in RUN mode, cannot start session.");
                break;
            }
            if (runtime_.deviceMode() == liftrr::core::MODE_RUN) {
                storage_.startSession(sid,
                                      "lift",
                                      sensors_.laserOffset(),
                                      sensors_.rollOffset(),
                                      sensors_.pitchOffset(),
                                      sensors_.yawOffset());
                Serial.println("Session started via serial 's'");
            }
            break;
        }

        case 'e':
            if (!storage_.isSessionActive()) {
                Serial.println("No active session to end.");
                break;
            }

            storage_.endSession();
            Serial.println("Session ended via serial 'e'");
            break;

        case 'i': {
            if (!storage_.initSd()) {
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
            sensors_.read(sample);
            Serial.print("cal: s="); Serial.print(sample.s);
            Serial.print(" g="); Serial.print(sample.g);
            Serial.print(" a="); Serial.print(sample.a);
            Serial.print(" m="); Serial.println(sample.m);
            Serial.print("laserValid="); Serial.print(sensors_.laserValid() ? "1" : "0");
            Serial.print(" dist="); Serial.println(sample.rawDist);
            Serial.print("isCalibrated="); Serial.println(sensors_.isCalibrated() ? "1" : "0");
            break;
        }

        default:
            // ignore unknown commands
            break;
    }
}

} // namespace app
} // namespace liftrr
