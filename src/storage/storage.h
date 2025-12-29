namespace liftrr {
namespace storage {

bool storageInitSd();

bool storageIsSessionActive();

bool storageStartSession(const String& sessionId,
                         const String& exercise,
                         int16_t calibLaserOffset,
                         float calibRollOffset,
                         float calibPitchOffset,
                         float calibYawOffset);

bool storageLogSample(int64_t timestampMs,
                      int16_t distMm,
                      int16_t relDistMm,
                      float rollDeg,
                      float pitchDeg,
                      float yawDeg);

bool storageEndSession();

typedef bool (*StorageSessionIndexCallback)(const char *name,
                                            uint32_t size,
                                            uint64_t mtimeMs,
                                            size_t lineIndex,
                                            void *ctx);
bool storageReadSessionIndex(size_t cursor,
                             size_t maxItems,
                             size_t *nextCursor,
                             bool *hasMore,
                             StorageSessionIndexCallback cb,
                             void *ctx);

bool storageRebuildSessionIndex(size_t *outCount);

// Lookup session in index; outName set to "<sessionId>.csv" or ".tmp".
bool storageFindSessionInIndex(const String &sessionId, String &outName);

} // namespace storage
} // namespace liftrr
