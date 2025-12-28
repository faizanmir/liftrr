#pragma once

#include <Arduino.h>
#include <string>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

namespace liftrr {
namespace ble {

// BLE error codes for initialization and runtime issues
enum class BleError : uint8_t {
    NONE = 0,
    INIT_FAILED,
    SERVER_CREATE_FAILED,
    SERVICE_CREATE_FAILED,
    COMMAND_CHAR_CREATE_FAILED,
    STATUS_CHAR_CREATE_FAILED,
    ADVERTISING_START_FAILED
};

// Configuration passed to BLE initialization
struct BleInitConfig {
    const char *deviceName;  // Shown in Android BT scanner
    uint16_t mtu;            // Desired MTU (e.g. 185 for ESP32)
};

// Application-level callbacks that your main code can implement
class BleCallbacks {
public:
    virtual ~BleCallbacks();

    // Raw command payload received from the phone over the COMMAND characteristic.
    // Implementation in bluetooth.cpp will be a no-op by default.
    virtual void onRawCommand(const std::string &raw);

    // Optional: connection state changes
    virtual void onConnected();
    virtual void onDisconnected();
};

// UUIDs for the LIFTRR BLE control service and its characteristics
extern const char *const LIFTRR_CONTROL_SERVICE_UUID;
extern const char *const LIFTRR_COMMAND_CHAR_UUID;
extern const char *const LIFTRR_STATUS_CHAR_UUID;

// BLE manager responsible for:
//  - Initializing the ESP32 BLE stack
//  - Creating GATT server, service and characteristics
//  - Handling connection callbacks
//  - Routing incoming COMMAND writes to BleCallbacks
//  - Providing helpers to send STATUS notifications back to the phone
class BleManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    BleManager();

    // Initialize BLE stack and start advertising. Returns BleError::NONE on success.
    BleError begin(const BleInitConfig &cfg, BleCallbacks *appCallbacks);

    // Non-blocking loop hook. Call from your main loop() if you later need
    // periodic work (timeouts, ping scheduling, etc.).
    void loop();

    // Get last error state and message (set when begin() or internals fail).
    BleError lastError() const;
    const String &lastErrorMessage() const;

    // Helper: send a status JSON (or any UTF-8 string) to the phone via notifications.
    // Returns false if not connected or STATUS characteristic not ready.
    bool sendStatus(const String &payload);

    // Convenience aliases – logically different, same transport for now.
    bool sendSessionsList(const String &payload);
    bool sendPong(const String &payload);

    bool isConnected() const;

protected:
    // BLEServerCallbacks
    void onConnect(BLEServer *pServer) override;
    void onDisconnect(BLEServer *pServer) override;

    // BLECharacteristicCallbacks – called when phone writes to COMMAND characteristic
    void onWrite(BLECharacteristic *pCharacteristic) override;

private:
    void setError(BleError err, const char *msg);
    void handleIncomingCommand(const std::string &data);

    BleError _lastError;
    String _lastErrorMessage;

    bool _isConnected;
    uint32_t _nextSeqNo; // reserved for future use if you add seq numbers in payload

    BleCallbacks *_appCallbacks;

    BLEServer *_server;
    BLEService *_controlService;
    BLECharacteristic *_commandChar;
    BLECharacteristic *_statusChar;
};

} // namespace ble
} // namespace liftrr
