#include <Arduino.h>
#include <ArduinoJson.h>

#include "core/config.h"
#include "storage/storage.h"

namespace liftrr {
namespace storage {

const char *const StorageManager::SESSION_INDEX_PATH = "/sessions/index.ndjson";
const char *const StorageManager::SESSIONS_DIR_PATH = "/sessions";

StorageManager::StorageManager(fs::SDFS &sd, void (*pulseFn)())
    : sd_(sd),
      pulse_fn_(pulseFn),
      sd_ready_(false),
      session_active_(false),
      last_sd_flush_ms_(0) {}

File StorageManager::openForAppend(const char *path) {
    File f = sd_.open(path, FILE_WRITE);
    if (f) {
        f.seek(f.size());
    }
    return f;
}

String StorageManager::basenameFromPath(const String &path) {
    int slash = path.lastIndexOf('/');
    if (slash < 0) return path;
    return path.substring(slash + 1);
}

uint64_t StorageManager::fileMtimeMs(File &) {
    return 0;
}

void StorageManager::pulseIndicator() const {
    if (pulse_fn_) pulse_fn_();
}

bool StorageManager::initSd() {
    if (sd_ready_) return true;

    if (!sd_.begin(SD_CS)) {
        Serial.println("SD init failed");
        sd_ready_ = false;
        return false;
    }

    sd_.mkdir(SESSIONS_DIR_PATH);

    sd_ready_ = true;
    Serial.println("SD init OK");
    return true;
}

bool StorageManager::isSessionActive() const {
    return session_active_;
}

bool StorageManager::startSession(const String &sessionId,
                                  const String &exercise,
                                  int16_t calibLaserOffset,
                                  float calibRollOffset,
                                  float calibPitchOffset,
                                  float calibYawOffset) {
    pulseIndicator();
    if (session_active_) {
        Serial.println("storageStartSession: session already active.");
        return false;
    }

    if (!initSd()) {
        return false;
    }

    current_session_id_ = sessionId;

    String dir = SESSIONS_DIR_PATH;
    sd_.mkdir(dir);

    if (!sd_.exists(SESSION_INDEX_PATH)) {
        File idx = sd_.open(SESSION_INDEX_PATH, FILE_WRITE);
        if (idx) idx.close();
    }

    String tmpPath = dir + "/" + sessionId + ".tmp";

    session_file_ = sd_.open(tmpPath, FILE_WRITE);
    if (!session_file_) {
        Serial.print("storageStartSession: failed to open ");
        Serial.println(tmpPath);
        current_session_id_ = "";
        return false;
    }

    session_file_.println("# liftrr session");
    session_file_.print("# session_id="); session_file_.println(sessionId);
    session_file_.print("# exercise=");   session_file_.println(exercise);
    session_file_.print("# calib_laserOffset="); session_file_.println(calibLaserOffset);
    session_file_.print("# calib_rollOffset=");  session_file_.println(calibRollOffset);
    session_file_.print("# calib_pitchOffset="); session_file_.println(calibPitchOffset);
    session_file_.print("# calib_yawOffset=");   session_file_.println(calibYawOffset);

    session_file_.println("timestamp_ms,dist_mm,relDist_mm,roll_deg,pitch_deg,yaw_deg");
    session_file_.flush();
    session_active_ = true;
    last_sd_flush_ms_ = millis();

    Serial.print("Session started: ");
    Serial.println(tmpPath);
    pulseIndicator();
    return true;
}

bool StorageManager::logSample(int64_t timestampMs,
                               int16_t distMm,
                               int16_t relDistMm,
                               float rollDeg,
                               float pitchDeg,
                               float yawDeg) {
    if (!session_active_ || !session_file_) return false;
    if (!sd_ready_) return false;
    pulseIndicator();
    session_file_.print((long long)timestampMs);
    session_file_.print(",");
    session_file_.print(distMm);
    session_file_.print(",");
    session_file_.print(relDistMm);
    session_file_.print(",");
    session_file_.print(rollDeg, 3);
    session_file_.print(",");
    session_file_.print(pitchDeg, 3);
    session_file_.print(",");
    session_file_.println(yawDeg, 3);

    unsigned long now = millis();
    if (now - last_sd_flush_ms_ > SD_FLUSH_INTERVAL_MS) {
        session_file_.flush();
        last_sd_flush_ms_ = now;
    }
    pulseIndicator();

    return true;
}

bool StorageManager::endSession() {
    pulseIndicator();
    if (!session_active_) {
        Serial.println("storageEndSession: no active session.");
        return false;
    }

    if (!sd_ready_) {
        session_active_ = false;
        current_session_id_ = "";
        return false;
    }

    if (session_file_) {
        session_file_.flush();
        session_file_.close();
    }

    String dir = "/sessions";
    String tmpPath   = dir + "/" + current_session_id_ + ".tmp";
    String finalPath = dir + "/" + current_session_id_ + ".csv";

    if (sd_.exists(tmpPath)) {
        if (!sd_.rename(tmpPath, finalPath)) {
            Serial.println("storageEndSession: rename failed, leaving .tmp file.");
        } else {
            Serial.print("Session finalized: ");
            Serial.println(finalPath);
        }
    }

    String indexPath;
    if (sd_.exists(finalPath)) indexPath = finalPath;
    else if (sd_.exists(tmpPath)) indexPath = tmpPath;

    if (indexPath.length()) {
        File f = sd_.open(indexPath, FILE_READ);
        if (f) {
            uint32_t size = f.size();
            uint64_t mtimeMs = fileMtimeMs(f);
            String name = basenameFromPath(indexPath);
            f.close();
            File idx = openForAppend(SESSION_INDEX_PATH);
            if (idx) {
                idx.print("{\"name\":\"");
                idx.print(name);
                idx.print("\",\"size\":");
                idx.print(size);
                idx.print(",\"mtime\":");
                idx.print((unsigned long long)mtimeMs);
                idx.println("}");
                idx.close();
            } else {
                Serial.println("storageEndSession: unable to append to index.");
            }
        }
    }

    session_active_ = false;
    current_session_id_ = "";
    pulseIndicator();
    return true;
}

bool StorageManager::readSessionIndex(size_t cursor,
                                      size_t maxItems,
                                      size_t *nextCursor,
                                      bool *hasMore,
                                      SessionIndexCallback cb,
                                      void *ctx) {
    if (nextCursor) *nextCursor = cursor;
    if (hasMore) *hasMore = false;
    if (!cb) return false;
    if (!initSd()) return false;

    if (!sd_.exists(SESSION_INDEX_PATH)) {
        return false;
    }

    File idx = sd_.open(SESSION_INDEX_PATH, FILE_READ);
    if (!idx) return false;

    size_t lineIndex = 0;
    size_t count = 0;
    size_t lastIncludedLine = cursor;

    while (idx.available()) {
        String line = idx.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
            lineIndex++;
            continue;
        }
        if (lineIndex < cursor) {
            lineIndex++;
            continue;
        }

        if (count >= maxItems) {
            if (hasMore) *hasMore = true;
            break;
        }

        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, line);
        if (err) {
            lastIncludedLine = lineIndex + 1;
            lineIndex++;
            continue;
        }

        const char *name = doc["name"] | "";
        uint32_t size = doc["size"] | 0;
        uint64_t mtimeMs = doc["mtime"] | (uint64_t)0;

        if (name[0] != '\0') {
            bool keepGoing = cb(name, size, mtimeMs, lineIndex, ctx);
            if (!keepGoing) {
                if (hasMore) *hasMore = true;
                break;
            }
            lastIncludedLine = lineIndex + 1;
            count++;
        }

        lineIndex++;
    }

    idx.close();
    if (nextCursor) *nextCursor = lastIncludedLine;
    return true;
}

bool StorageManager::findSessionInIndex(const String &sessionId, String &outName) {
    outName = "";
    if (!initSd()) return false;
    if (!sd_.exists(SESSION_INDEX_PATH)) return false;

    File idx = sd_.open(SESSION_INDEX_PATH, FILE_READ);
    if (!idx) return false;

    String wantCsv = sessionId + ".csv";
    String wantTmp = sessionId + ".tmp";
    String foundTmp;

    while (idx.available()) {
        String line = idx.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) {
            continue;
        }
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, line);
        if (err) continue;

        const char *name = doc["name"] | "";
        if (name[0] == '\0') continue;

        if (wantCsv.equals(name)) {
            outName = name;
            idx.close();
            return true;
        }
        if (wantTmp.equals(name)) {
            foundTmp = name;
        }
    }

    idx.close();
    if (foundTmp.length()) {
        outName = foundTmp;
        return true;
    }
    return false;
}

bool StorageManager::rebuildSessionIndex(size_t *outCount) {
    if (outCount) *outCount = 0;
    if (!initSd()) return false;

    File dir = sd_.open(SESSIONS_DIR_PATH);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return false;
    }

    File idx = sd_.open(SESSION_INDEX_PATH, FILE_WRITE);
    if (!idx) {
        dir.close();
        return false;
    }

    size_t count = 0;
    File entry = dir.openNextFile();
    while (entry) {
        if (!entry.isDirectory()) {
            const char *rawName = entry.name();
            String baseName = basenameFromPath(String(rawName));
            if (baseName != "index.ndjson") {
                bool isSessionFile = baseName.endsWith(".csv") || baseName.endsWith(".tmp");
                if (isSessionFile) {
                    uint32_t size = entry.size();
                    uint64_t mtimeMs = fileMtimeMs(entry);
                    idx.print("{\"name\":\"");
                    idx.print(baseName);
                    idx.print("\",\"size\":");
                    idx.print(size);
                    idx.print(",\"mtime\":");
                    idx.print((unsigned long long)mtimeMs);
                    idx.println("}");
                    count++;
                }
            }
        }
        entry.close();
        entry = dir.openNextFile();
    }

    idx.close();
    dir.close();
    if (outCount) *outCount = count;
    return true;
}

} // namespace storage
} // namespace liftrr
