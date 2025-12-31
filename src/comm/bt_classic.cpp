#include "comm/bt_classic.h"

#include <SD.h>
#include <ArduinoJson.h>
#include "bt_classic.h"

namespace liftrr {
namespace comm {

BtClassicManager::BtClassicManager(fs::SDFS &sd)
    : sd_(sd), bt_ready_(false) {}

void BtClassicManager::sendEventLine(const char *eventName,
                                     const String &sessionId,
                                     size_t size) {
    JsonDocument doc;
    doc["event"] = eventName;
    doc["sessionId"] = sessionId;
    doc["size"] = (uint32_t)size;
    String line;
    serializeJson(doc, line);
    bt_serial_.println(line);
}

bool BtClassicManager::init(const char *deviceName) {
    if (bt_ready_) return true;
    if (!deviceName || deviceName[0] == '\0') {
        deviceName = "LIFTRR";
    }
    bt_ready_ = bt_serial_.begin(deviceName);
    return bt_ready_;
}

bool BtClassicManager::isConnected() {
    return bt_ready_ && bt_serial_.hasClient();
}

bool BtClassicManager::startFileStream(const String &path,
                                       size_t size,
                                       const String &sessionId) {
    if (!isConnected()) return false;
    if (stream_.active) return false;
    if (!sd_.exists(path)) return false;

    File f = sd_.open(path, FILE_READ);
    if (!f) return false;

    stream_.file = f;
    stream_.active = true;
    stream_.offset = 0;
    stream_.size = size;
    stream_.sessionId = sessionId;

    Serial.print("[BT] Stream start: ");
    Serial.print(path);
    Serial.print(" bytes=");
    Serial.println(size);

    return true;
}

bool BtClassicManager::sendJsonLine(const String &line) {
    if (!isConnected()) return false;
    bt_serial_.println(line);
    return true;
}

void BtClassicManager::loop() {
    if (!stream_.active) return;
    if (!isConnected()) {
        if (stream_.file) stream_.file.close();
        stream_ = BtStreamState{};
        return;
    }

    if (!stream_.file) {
        stream_ = BtStreamState{};
        return;
    }

    const size_t kChunkSize = 512;
    uint8_t buf[kChunkSize];
    size_t n = stream_.file.read(buf, kChunkSize);
    if (n > 0) {
        bt_serial_.write(buf, n);
        stream_.offset += n;
    }

    if (n == 0 || stream_.offset >= stream_.size || !stream_.file.available()) {
        if (stream_.file) stream_.file.close();
        Serial.print("[BT] Stream end: sessionId=");
        Serial.print(stream_.sessionId);
        Serial.print(" bytes=");
        Serial.println(stream_.offset);
        stream_ = BtStreamState{};
    }
}

} // namespace comm
} // namespace liftrr
