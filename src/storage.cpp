#include <Arduino.h>
#include <SPI.h>

#include "globals.h"
#include "storage.h"

// ==============================
//  W25Qxx low-level definitions
// ==============================

static const uint8_t W25_CMD_READ_DATA       = 0x03;
static const uint8_t W25_CMD_PAGE_PROGRAM    = 0x02;
static const uint8_t W25_CMD_SECTOR_ERASE_4K = 0x20;
static const uint8_t W25_CMD_READ_STATUS1    = 0x05;
static const uint8_t W25_CMD_WRITE_ENABLE    = 0x06;
static const uint8_t W25_CMD_READ_JEDEC_ID   = 0x9F;

static const uint32_t W25_SECTOR_SIZE  = 4096;
static const uint32_t W25_PAGE_SIZE    = 256;

// Log region configuration: start at 0, use first 64 KiB
static const uint32_t LOG_REGION_START = 0x000000;
static const uint32_t LOG_REGION_SIZE  = 64 * 1024; // 64KB for CSV text

// Current write pointer in log region
static uint32_t flashWriteAddr = LOG_REGION_START;

// ==============================
//  Chip select helpers
// ==============================

static inline void w25Select() {
  digitalWrite(FLASH_CS, LOW);
}

static inline void w25Deselect() {
  digitalWrite(FLASH_CS, HIGH);
}

// ==============================
//  Low-level SPI helpers
// ==============================

static uint8_t w25ReadStatus1() {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  w25Select();
  SPI.transfer(W25_CMD_READ_STATUS1);
  uint8_t status = SPI.transfer(0x00);
  w25Deselect();
  SPI.endTransaction();
  return status;
}

static void w25WaitBusy() {
  // BUSY bit = bit0 in status1
  while (w25ReadStatus1() & 0x01) {
    delay(1);
  }
}

static void w25WriteEnable() {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  w25Select();
  SPI.transfer(W25_CMD_WRITE_ENABLE);
  w25Deselect();
  SPI.endTransaction();
}

static void w25ReadJedecID(uint8_t &manuf, uint8_t &memType, uint8_t &capacity) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  w25Select();
  SPI.transfer(W25_CMD_READ_JEDEC_ID);
  manuf    = SPI.transfer(0x00);
  memType  = SPI.transfer(0x00);
  capacity = SPI.transfer(0x00);
  w25Deselect();
  SPI.endTransaction();
}

static void w25SectorErase(uint32_t addr) {
  // addr must be 4K aligned
  w25WriteEnable();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  w25Select();
  SPI.transfer(W25_CMD_SECTOR_ERASE_4K);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8)  & 0xFF);
  SPI.transfer((addr >> 0)  & 0xFF);
  w25Deselect();
  SPI.endTransaction();
  w25WaitBusy();
}

static void w25PageProgram(uint32_t addr, const uint8_t *data, size_t len) {
  if (len == 0 || len > W25_PAGE_SIZE) return;

  w25WriteEnable();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  w25Select();
  SPI.transfer(W25_CMD_PAGE_PROGRAM);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8)  & 0xFF);
  SPI.transfer((addr >> 0)  & 0xFF);

  for (size_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }

  w25Deselect();
  SPI.endTransaction();
  w25WaitBusy();
}

static void w25ReadData(uint32_t addr, uint8_t *buf, size_t len) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  w25Select();
  SPI.transfer(W25_CMD_READ_DATA);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8)  & 0xFF);
  SPI.transfer((addr >> 0)  & 0xFF);

  for (size_t i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }

  w25Deselect();
  SPI.endTransaction();
}

// Multi-page program helper: splits writes across page boundaries
static void w25WriteBuffer(uint32_t addr, const uint8_t *data, size_t len) {
  while (len > 0) {
    uint32_t pageOffset   = addr % W25_PAGE_SIZE;
    uint32_t spaceInPage  = W25_PAGE_SIZE - pageOffset;
    size_t   chunk        = (len < spaceInPage) ? len : spaceInPage;

    w25PageProgram(addr, data, chunk);

    addr += chunk;
    data += chunk;
    len  -= chunk;
  }
}

// ==============================
//  Log region management
// ==============================

// Scan log region to find first 0xFF (erased byte) as append point.
// Assumes region was fully erased before first use and that we only write ASCII CSV.
static void findWritePointer() {
  uint8_t buf[64];
  flashWriteAddr = LOG_REGION_START;

  for (uint32_t addr = LOG_REGION_START; addr < LOG_REGION_START + LOG_REGION_SIZE; addr += sizeof(buf)) {
    size_t blockSize = sizeof(buf);
    if (addr + blockSize > LOG_REGION_START + LOG_REGION_SIZE) {
      blockSize = (LOG_REGION_START + LOG_REGION_SIZE) - addr;
    }

    w25ReadData(addr, buf, blockSize);

    bool allFF = true;
    for (size_t i = 0; i < blockSize; i++) {
      if (buf[i] != 0xFF) {
        allFF = false;
      } else {
        // First erased byte: this is our append point
        flashWriteAddr = addr + i;
        return;
      }
    }

    if (allFF) {
      // This whole block is empty, use its start as append point
      flashWriteAddr = addr;
      return;
    }
  }

  // Region full
  flashWriteAddr = LOG_REGION_START + LOG_REGION_SIZE;
}

// ==============================
//  Lifecycle
// ==============================

bool initFlashStorage() {
  Serial.println("Init external flash (raw W25Qxx CSV logger)...");

  pinMode(FLASH_CS, OUTPUT);
  w25Deselect();

  // Init SPI bus for flash
  SPI.begin(FLASH_SCK, FLASH_MISO, FLASH_MOSI, FLASH_CS);

  uint8_t manuf, memType, capacity;
  w25ReadJedecID(manuf, memType, capacity);
  Serial.print("JEDEC ID: 0x");
  Serial.print(manuf, HEX);
  Serial.print(" 0x");
  Serial.print(memType, HEX);
  Serial.print(" 0x");
  Serial.println(capacity, HEX);

  // Determine current write pointer by scanning region
  findWritePointer();

  Serial.print("Log region start: 0x");
  Serial.print(LOG_REGION_START, HEX);
  Serial.print(" size: ");
  Serial.print(LOG_REGION_SIZE);
  Serial.print(" bytes, next write at 0x");
  Serial.println(flashWriteAddr, HEX);

  flashReady = true;
  return true;
}

bool flashFactoryReset() {
  Serial.println("FLASH FACTORY RESET: erasing log region...");

  pinMode(FLASH_CS, OUTPUT);
  w25Deselect();
  SPI.begin(FLASH_SCK, FLASH_MISO, FLASH_MOSI, FLASH_CS);

  // Erase all sectors in log region
  for (uint32_t addr = LOG_REGION_START; addr < LOG_REGION_START + LOG_REGION_SIZE; addr += W25_SECTOR_SIZE) {
    Serial.print("Erasing sector at 0x");
    Serial.println(addr, HEX);
    w25SectorErase(addr);
  }

  flashWriteAddr = LOG_REGION_START;
  flashReady = true;
  Serial.println("Factory reset complete. Log region cleared.");
  return true;
}

// ==============================
//  Logging primitives
// ==============================

bool flashAppend(const void* data, size_t len) {
  if (!flashReady && !initFlashStorage()) {
    Serial.println("flashAppend: flash not ready.");
    return false;
  }

  if (len == 0) {
    Serial.println("flashAppend: zero-length write, nothing to do.");
    return true;
  }

  if (flashWriteAddr + len > LOG_REGION_START + LOG_REGION_SIZE) {
    Serial.println("flashAppend: log region full, cannot append.");
    return false;
  }

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
  w25WriteBuffer(flashWriteAddr, bytes, len);

  Serial.print("flashAppend: wrote ");
  Serial.print(len);
  Serial.print(" bytes at addr 0x");
  Serial.println(flashWriteAddr, HEX);

  flashWriteAddr += len;
  return true;
}

bool logLiftSampleCsv(const LiftSample& sample) {
  // Build CSV line: t_ms,dist_mm,roll_deg,pitch_deg,yaw_deg\n
  char line[128];
  int n = snprintf(
    line,
    sizeof(line),
    "%lu,%d,%.2f,%.2f,%.2f\n",
    static_cast<unsigned long>(sample.t),
    static_cast<int>(sample.dist),
    sample.roll,
    sample.pitch,
    sample.yaw
  );

  if (n <= 0) {
    Serial.println("logLiftSampleCsv: Failed to format CSV line.");
    return false;
  }
  if (n >= (int)sizeof(line)) {
    Serial.println("logLiftSampleCsv: CSV line truncated.");
    n = sizeof(line) - 1;
    line[n] = '\n';
  }

  return flashAppend(line, (size_t)n);
}

bool writeDummySampleToFlash() {
  if (!flashReady && !initFlashStorage()) {
    Serial.println("writeDummySampleToFlash: flash not ready.");
    return false;
  }

  LiftSample s;
  s.t     = millis();
  s.dist  = distance;
  s.roll  = rollOffset;
  s.pitch = pitchOffset;
  s.yaw   = yawOffset;

  if (!logLiftSampleCsv(s)) {
    Serial.println("writeDummySampleToFlash: logLiftSampleCsv FAILED.");
    return false;
  }

  Serial.println("writeDummySampleToFlash: CSV line logged.");
  return true;
}

// ==============================
//  Reading utilities
// ==============================

bool readLogChunk(uint32_t addr, uint8_t* buf, size_t len) {
  if (!flashReady && !initFlashStorage()) {
    Serial.println("readLogChunk: flash not ready.");
    return false;
  }

  if (addr + len > LOG_REGION_SIZE) {
    Serial.println("readLogChunk: request out of bounds.");
    return false;
  }

  w25ReadData(LOG_REGION_START + addr, buf, len);
  return true;
}