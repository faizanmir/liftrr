#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <FS.h>
#include <SD.h>

namespace liftrr {
namespace comm {

class BtClassicManager {
public:
    explicit BtClassicManager(fs::SDFS &sd);

    bool init(const char *deviceName);
    bool isConnected();
    bool startFileStream(const String &path,
                         size_t size,
                         const String &sessionId);
    bool sendJsonLine(const String &line);
    void loop();

private:
    struct BtStreamState {
        bool active = false;
        File file;
        size_t offset = 0;
        size_t size = 0;
        String sessionId;
    };

    void sendEventLine(const char *eventName,
                       const String &sessionId,
                       size_t size);

    BluetoothSerial bt_serial_;
    fs::SDFS &sd_;
    bool bt_ready_;
    BtStreamState stream_;
};

} // namespace comm
} // namespace liftrr
