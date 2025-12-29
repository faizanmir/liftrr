#include "comm/bt_classic.h"

#include <BluetoothSerial.h>
#include <SD.h>
#include <ArduinoJson.h>

namespace liftrr {
namespace comm {

static BluetoothSerial gBtSerial;
static bool gBtReady = false;

struct BtStreamState {
    bool active = false;
    File file;
    size_t offset = 0;
    size_t size = 0;
    String sessionId;
};

static BtStreamState gStream;

static void sendEventLine(const char *eventName,
                          const String &sessionId,
                          size_t size) {
    JsonDocument doc;
    doc["event"] = eventName;
    doc["sessionId"] = sessionId;
    doc["size"] = (uint32_t)size;
    String line;
    serializeJson(doc, line);
    gBtSerial.println(line);
}

bool btClassicInit(const char *deviceName) {
    if (gBtReady) return true;
    if (!deviceName || deviceName[0] == '\0') {
        deviceName = "LIFTRR";
    }
    gBtReady = gBtSerial.begin(deviceName);
    return gBtReady;
}

bool btClassicIsConnected() {
    return gBtReady && gBtSerial.hasClient();
}

bool btClassicStartFileStream(const String &path,
                              size_t size,
                              const String &sessionId) {
    if (!btClassicIsConnected()) return false;
    if (gStream.active) return false;
    if (!SD.exists(path)) return false;

    File f = SD.open(path, FILE_READ);
    if (!f) return false;

    gStream.file = f;
    gStream.active = true;
    gStream.offset = 0;
    gStream.size = size;
    gStream.sessionId = sessionId;

    sendEventLine("session.file.start", sessionId, size);
    return true;
}

void btClassicLoop() {
    if (!gStream.active) return;
    if (!btClassicIsConnected()) {
        if (gStream.file) gStream.file.close();
        gStream = BtStreamState{};
        return;
    }

    if (!gStream.file) {
        gStream = BtStreamState{};
        return;
    }

    const size_t kChunkSize = 512;
    uint8_t buf[kChunkSize];
    size_t n = gStream.file.read(buf, kChunkSize);
    if (n > 0) {
        gBtSerial.write(buf, n);
        gStream.offset += n;
    }

    if (n == 0 || gStream.offset >= gStream.size || !gStream.file.available()) {
        if (gStream.file) gStream.file.close();
        sendEventLine("session.file.end", gStream.sessionId, gStream.offset);
        gStream = BtStreamState{};
    }
}

} // namespace comm
} // namespace liftrr
