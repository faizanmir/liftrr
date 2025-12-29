#include "ble.h"

namespace liftrr {
namespace ble {

// UUID definitions.

const char *const LIFTRR_CONTROL_SERVICE_UUID =
    "c7ccb0e4-7cd7-45f6-9693-b3dda2d77672";
const char *const LIFTRR_COMMAND_CHAR_UUID =
    "b0f2a8fb-9a63-4d9f-8ffc-80501fda2359";
const char *const LIFTRR_STATUS_CHAR_UUID =
    "27746aa3-5fae-44c1-a1eb-65844cc315dc";

// BleCallbacks defaults.

BleCallbacks::~BleCallbacks() = default;

void BleCallbacks::onRawCommand(const std::string &raw) {
    (void)raw; // default no-op
}

void BleCallbacks::onConnected() {
    // default no-op
}

void BleCallbacks::onDisconnected() {
    // default no-op
}

// BleManager implementation.

BleManager::BleManager()
    : _lastError(BleError::NONE),
      _lastErrorMessage(""),
      _isConnected(false),
      _nextSeqNo(1),
      _appCallbacks(nullptr),
      _server(nullptr),
      _controlService(nullptr),
      _commandChar(nullptr),
      _statusChar(nullptr) {}

void BleManager::setError(BleError err, const char *msg) {
    _lastError = err;
    _lastErrorMessage = msg ? msg : "";
}

BleError BleManager::begin(const BleInitConfig &cfg, BleCallbacks *appCallbacks) {
    _appCallbacks = appCallbacks;
    _lastError = BleError::NONE;
    _lastErrorMessage = "";

    const char *name = (cfg.deviceName && cfg.deviceName[0] != '\0')
                           ? cfg.deviceName
                           : "LIFTRR";

    // Initialize the BLE stack
    BLEDevice::init(name);

    if (cfg.mtu > 0) {
        BLEDevice::setMTU(cfg.mtu);
    }

    _server = BLEDevice::createServer();
    if (_server == nullptr) {
        setError(BleError::SERVER_CREATE_FAILED, "Failed to create BLE server");
        return _lastError;
    }

    _server->setCallbacks(this);

    _controlService = _server->createService(LIFTRR_CONTROL_SERVICE_UUID);
    if (_controlService == nullptr) {
        setError(BleError::SERVICE_CREATE_FAILED, "Failed to create control service");
        return _lastError;
    }

    // COMMAND characteristic – phone writes commands here
    _commandChar = _controlService->createCharacteristic(
        LIFTRR_COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );

    if (_commandChar == nullptr) {
        setError(BleError::COMMAND_CHAR_CREATE_FAILED,
                 "Failed to create command characteristic");
        return _lastError;
    }

    _commandChar->setCallbacks(this);

    // STATUS characteristic – ESP32 notifies phone with status / responses
    _statusChar = _controlService->createCharacteristic(
        LIFTRR_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    if (_statusChar == nullptr) {
        setError(BleError::STATUS_CHAR_CREATE_FAILED,
                 "Failed to create status characteristic");
        return _lastError;
    }

    // CCCD descriptor so Android can enable notifications
    _statusChar->addDescriptor(new BLE2902());

    _controlService->start();

    // Start advertising the control service
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    if (advertising == nullptr) {
        setError(BleError::ADVERTISING_START_FAILED,
                 "Advertising object is null");
        return _lastError;
    }

    advertising->addServiceUUID(LIFTRR_CONTROL_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);  // Recommended for better iOS/Android behavior
    advertising->setMaxPreferred(0x12);

    BLEDevice::startAdvertising();

    _lastError = BleError::NONE;
    _lastErrorMessage = "";
    return _lastError;
}

void BleManager::loop() {
    // Currently nothing required here – BLE stack is event-driven.
    // Keep this for future timeouts / periodic tasks if needed.
}

BleError BleManager::lastError() const {
    return _lastError;
}

const String &BleManager::lastErrorMessage() const {
    return _lastErrorMessage;
}

bool BleManager::sendStatus(const String &payload) {
    if (!_statusChar || !_isConnected) {
        return false;
    }
    Serial.println("[BLE] Sending status:");
    Serial.println(payload);
    _statusChar->setValue(payload.c_str());
    _statusChar->notify();
    return true;
}

bool BleManager::sendSessionsList(const String &payload) {
    return sendStatus(payload);
}

bool BleManager::sendPong(const String &payload) {
    return sendStatus(payload);
}

bool BleManager::isConnected() const {
    return _isConnected;
}

// BLE callbacks.

void BleManager::onConnect(BLEServer *pServer) {
    (void)pServer;
    _isConnected = true;
    if (_appCallbacks) {
        _appCallbacks->onConnected();
    }
}

void BleManager::onDisconnect(BLEServer *pServer) {
    (void)pServer;
    _isConnected = false;

    if (_appCallbacks) {
        _appCallbacks->onDisconnected();
    }

    // Restart advertising so the phone can reconnect
    BLEDevice::startAdvertising();
}

void BleManager::handleIncomingCommand(const std::string &data) {
    if (_appCallbacks) {
        _appCallbacks->onRawCommand(data);
    }
}

void BleManager::onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic != _commandChar) {
        return;
    }

    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
        handleIncomingCommand(value);
    }
}

} // namespace ble
} // namespace liftrr
