#pragma once
// ============================================================
//  ota_manager.h  –  OTA firmware + filesystem update v2.0
//
//  New in v2.0:
//    + Pre-OTA flash free space validation (rejects oversized FW)
//    + LittleFS temp-file cleanup before OTA starts
//    + Firmware size check: rejects if > OTA_MAX_FIRMWARE_BYTES
//    + Boot partition rollback on end() failure
//    + Real progress percentage (bytes written / partition size)
//    + Chunk watchdog: abort stale uploads (OTA_CHUNK_TIMEOUT_MS)
//    + Concurrent upload guard (aborts stale session cleanly)
//    + IR ISR paused during flash writes
//    + LittleFS re-mounted after abort so config saves keep working
// ============================================================
#include <Arduino.h>
#include <Update.h>
#include <LittleFS.h>
#include <functional>
#include "config.h"

// Abort if no chunk arrives within 60 s (dropped TCP connection)
#ifndef OTA_CHUNK_TIMEOUT_MS
  #define OTA_CHUNK_TIMEOUT_MS  60000UL
#endif

// Maximum accepted firmware binary size (bytes).
// Slightly less than partition size (1408 KB) to leave margin.
// Reject at 1380 KB to avoid Update.begin() failing mid-transfer.
#ifndef OTA_MAX_FIRMWARE_BYTES
  #define OTA_MAX_FIRMWARE_BYTES  (1380UL * 1024UL)
#endif

using OtaProgressCallback = std::function<void(size_t done, size_t total)>;
using OtaEndCallback      = std::function<void(bool success, const String& msg)>;

class OtaManager {
public:
    OtaManager();

    void onProgress(OtaProgressCallback cb) { _progressCb = cb; }
    void onEnd     (OtaEndCallback      cb) { _endCb      = cb; }

    // Called per-chunk from web server upload handler.
    // target = "firmware" | "filesystem"
    void handleUploadChunk(const String& target,
                           uint8_t*      data,
                           size_t        len,
                           size_t        index,
                           size_t        total,   // total file size (0 = unknown)
                           bool          final);

    bool          isUpdating()     const { return _updating; }
    bool          restartPending() const { return _restartPending; }
    const String& lastError()      const { return _lastError; }

    // Call from main loop() — aborts stale uploads
    void tickWatchdog();

    // Clear error state without reboot — allows retrying OTA
    void clearError();

    // Free OTA space in bytes (inactive partition)
    size_t freeOtaBytes() const;

private:
    OtaProgressCallback _progressCb;
    OtaEndCallback      _endCb;
    volatile bool       _updating;
    volatile bool       _restartPending;
    String              _lastError;
    uint8_t             _lastPct;
    unsigned long       _lastChunkMs;
    size_t              _totalReceived;
    size_t              _declaredSize;   // total bytes declared by HTTP Content-Length

    void beginUpdate  (const String& target, size_t declaredSize);
    void finishUpdate ();
    void abortUpdate  (const String& reason);
    bool _cleanTempFiles();              // remove LittleFS temp/log files pre-OTA
    bool _validateFlashSpace(size_t declaredSize, const String& target);
};

extern OtaManager otaMgr;
