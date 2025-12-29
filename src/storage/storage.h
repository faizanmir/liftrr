#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SD.h>

namespace liftrr {
namespace storage {

class StorageManager {
public:
    typedef bool (*SessionIndexCallback)(const char *name,
                                         uint32_t size,
                                         uint64_t mtimeMs,
                                         size_t lineIndex,
                                         void *ctx);

    explicit StorageManager(fs::SDFS &sd, void (*pulseFn)() = nullptr);

    bool initSd();
    bool isSessionActive() const;

    bool startSession(const String &sessionId,
                      const String &exercise,
                      int16_t calibLaserOffset,
                      float calibRollOffset,
                      float calibPitchOffset,
                      float calibYawOffset);

    bool logSample(int64_t timestampMs,
                   int16_t distMm,
                   int16_t relDistMm,
                   float rollDeg,
                   float pitchDeg,
                   float yawDeg);

    bool endSession();

    bool readSessionIndex(size_t cursor,
                          size_t maxItems,
                          size_t *nextCursor,
                          bool *hasMore,
                          SessionIndexCallback cb,
                          void *ctx);

    bool rebuildSessionIndex(size_t *outCount);

    bool findSessionInIndex(const String &sessionId, String &outName);

private:
    File openForAppend(const char *path);
    String basenameFromPath(const String &path);
    uint64_t fileMtimeMs(File &file);
    void pulseIndicator() const;

    fs::SDFS &sd_;
    void (*pulse_fn_)();
    bool sd_ready_;
    bool session_active_;
    File session_file_;
    String current_session_id_;
    unsigned long last_sd_flush_ms_;

    static const unsigned long SD_FLUSH_INTERVAL_MS = 1000;
    static const char *const SESSION_INDEX_PATH;
    static const char *const SESSIONS_DIR_PATH;
};

} // namespace storage
} // namespace liftrr
