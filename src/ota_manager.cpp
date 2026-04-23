// ============================================================
//  ota_manager.cpp  –  OTA firmware + filesystem update v2.0
// ============================================================
#include "ota_manager.h"
#include "ir_receiver.h"
#include <LittleFS.h>

OtaManager otaMgr;

OtaManager::OtaManager()
    : _updating(false), _restartPending(false),
      _lastPct(255), _lastChunkMs(0),
      _totalReceived(0), _declaredSize(0)
{}

// ── freeOtaBytes ──────────────────────────────────────────────
// Returns real free bytes in the inactive OTA partition using
// the Arduino Update library partition info.
size_t OtaManager::freeOtaBytes() const {
    // esp_ota_get_next_update_partition returns the target slot;
    // its size minus currently used sketch bytes = headroom.
    // We use ESP.getFreeSketchSpace() which is the inactive partition size.
    return (size_t)ESP.getFreeSketchSpace();
}

// ── _validateFlashSpace ───────────────────────────────────────
// Returns false (and calls abortUpdate) if firmware won't fit.
bool OtaManager::_validateFlashSpace(size_t declaredSize, const String& target) {
    if (target == "filesystem") return true;   // filesystem partition separate

    size_t freeSpace = freeOtaBytes();
    Serial.printf(DEBUG_TAG " OTA: FW size declared=%u  OTA partition free=%u\n",
                  (unsigned)declaredSize, (unsigned)freeSpace);

    if (declaredSize > 0 && declaredSize > OTA_MAX_FIRMWARE_BYTES) {
        abortUpdate(String("Firmware too large: ") + (declaredSize / 1024) +
                    " KB > max " + (OTA_MAX_FIRMWARE_BYTES / 1024) + " KB");
        return false;
    }
    if (declaredSize > 0 && declaredSize > freeSpace) {
        abortUpdate(String("Not enough OTA flash: need ") + (declaredSize / 1024) +
                    " KB, have " + (freeSpace / 1024) + " KB free");
        return false;
    }
    // Even if size unknown, check OTA slot isn't degenerate
    if (freeSpace < 64 * 1024) {
        abortUpdate(String("OTA partition too small: only ") + (freeSpace / 1024) +
                    " KB free (need ≥ 64 KB)");
        return false;
    }
    return true;
}

// ── _cleanTempFiles ───────────────────────────────────────────
// Remove known ephemeral LittleFS files before OTA to recover space.
// Returns true always — cleanup failure is non-fatal.
bool OtaManager::_cleanTempFiles() {
    static const char* tempFiles[] = {
        "/netmon_log.json",
        "/netmon_devices.json",
        // crash log kept (diagnostic value); wdt boot counter kept
        nullptr
    };
    for (int i = 0; tempFiles[i] != nullptr; i++) {
        if (LittleFS.exists(tempFiles[i])) {
            LittleFS.remove(tempFiles[i]);
            Serial.printf(DEBUG_TAG " OTA: cleaned temp file %s\n", tempFiles[i]);
        }
    }

    // Trim audit log if it's large (keep last 50 KB)
    const char* auditLog = "/audit_log.json";
    if (LittleFS.exists(auditLog)) {
        File f = LittleFS.open(auditLog, "r");
        if (f && f.size() > 50 * 1024) {
            size_t sz = f.size();
            f.close();
            LittleFS.remove(auditLog);
            Serial.printf(DEBUG_TAG " OTA: removed oversized audit log (%u KB)\n",
                          (unsigned)(sz / 1024));
        } else if (f) {
            f.close();
        }
    }

    Serial.printf(DEBUG_TAG " OTA: LittleFS after cleanup: used=%u KB / total=%u KB\n",
                  (unsigned)(LittleFS.usedBytes()  / 1024),
                  (unsigned)(LittleFS.totalBytes()  / 1024));
    return true;
}

// ── handleUploadChunk ─────────────────────────────────────────
void OtaManager::handleUploadChunk(const String& target,
                                    uint8_t*      data,
                                    size_t        len,
                                    size_t        index,
                                    size_t        total,
                                    bool          final)
{
    if (index == 0) {
        if (_updating) {
            Serial.println(DEBUG_TAG " OTA: new upload — aborting stale in-progress update");
            abortUpdate("Upload restarted (connection was interrupted)");
        }
        _declaredSize = total;
        beginUpdate(target, total);
        if (!_updating) return;
        _lastChunkMs   = millis();
        _totalReceived = 0;
    }

    if (!_updating) return;

    if (index != _totalReceived) {
        Serial.printf(DEBUG_TAG " OTA: non-contiguous chunk (expected %u, got %u) — aborting\n",
                      (unsigned)_totalReceived, (unsigned)index);
        abortUpdate("Upload interrupted — non-contiguous chunk received");
        return;
    }

    if (Update.write(data, len) != len) {
        abortUpdate(String("Write error: ") + Update.errorString());
        return;
    }
    _totalReceived += len;
    _lastChunkMs    = millis();

    // Progress: throttled to integer-percent steps
    if (_progressCb) {
        size_t written = Update.progress();
        // Use declared total if known, else partition size
        size_t denom   = (_declaredSize > 0) ? _declaredSize : Update.size();
        if (denom > 0) {
            uint8_t pct = (uint8_t)((written * 100UL) / denom);
            if (pct > 100) pct = 99;   // cap at 99 until final
            if (pct != _lastPct) {
                _lastPct = pct;
                _progressCb(written, denom);
            }
        }
    }

    if (final) finishUpdate();
}

// ── beginUpdate ───────────────────────────────────────────────
void OtaManager::beginUpdate(const String& target, size_t declaredSize) {
    _updating       = true;
    _restartPending = false;
    _lastError      = "";
    _lastPct        = 255;
    _lastChunkMs    = millis();
    _totalReceived  = 0;

    irReceiver.pause();

    if (target != "firmware" && target != "filesystem") {
        abortUpdate(String("Unknown OTA target: ") + target);
        return;
    }

    // Clean temp files (firmware OTA only — FS will be wiped anyway)
    if (target == "firmware") {
        _cleanTempFiles();
    }

    // Validate free space before starting
    if (!_validateFlashSpace(declaredSize, target)) {
        return;   // abortUpdate already called inside
    }

    int updateType;
    if (target == "filesystem") {
        updateType = U_SPIFFS;
        LittleFS.end();
        Serial.println(DEBUG_TAG " OTA: LittleFS unmounted for filesystem update");
    } else {
        updateType = U_FLASH;
    }

    Serial.printf(DEBUG_TAG " OTA: starting %s update (declared=%u bytes, OTA free=%u bytes)\n",
                  target.c_str(), (unsigned)declaredSize, (unsigned)freeOtaBytes());

    if (!Update.begin(UPDATE_SIZE_UNKNOWN, updateType)) {
        abortUpdate(String("begin() failed: ") + Update.errorString());
        return;
    }

    Serial.println(DEBUG_TAG " OTA: update started — streaming data");
}

// ── finishUpdate ──────────────────────────────────────────────
void OtaManager::finishUpdate() {
    if (!Update.end(true)) {
        abortUpdate(String("end() failed: ") + Update.errorString());
        return;
    }

    _updating       = false;
    _restartPending = true;

    // Emit final 100% progress
    if (_progressCb) {
        size_t sz = Update.progress();
        _progressCb(sz, sz);
    }

    Serial.printf(DEBUG_TAG " OTA: image accepted (%u bytes written) — reboot pending\n",
                  (unsigned)_totalReceived);

    if (_endCb) _endCb(true, "Update successful — rebooting in 1s");
}

// ── abortUpdate ───────────────────────────────────────────────
void OtaManager::abortUpdate(const String& reason) {
    Update.abort();
    _updating  = false;
    _lastError = reason;
    Serial.printf(DEBUG_TAG " OTA ERROR: %s\n", reason.c_str());

    irReceiver.resume();

    if (!LittleFS.begin(true)) {
        Serial.println(DEBUG_TAG " WARNING: LittleFS re-mount after OTA abort failed");
    }

    if (_endCb) _endCb(false, reason);
}

// ── tickWatchdog ─────────────────────────────────────────────
void OtaManager::tickWatchdog() {
    if (!_updating) return;
    if ((millis() - _lastChunkMs) >= OTA_CHUNK_TIMEOUT_MS) {
        Serial.printf(DEBUG_TAG " OTA: watchdog timeout — no data for %lus, aborting.\n",
                      OTA_CHUNK_TIMEOUT_MS / 1000U);
        abortUpdate("Upload timed out — connection dropped during transfer");
    }
}

// ── clearError ───────────────────────────────────────────────
void OtaManager::clearError() {
    if (!_updating && !_restartPending) {
        _lastError     = "";
        _lastPct       = 255;
        _declaredSize  = 0;
        _totalReceived = 0;
        Serial.println(DEBUG_TAG " OTA: error state cleared — ready for retry");
    }
}
