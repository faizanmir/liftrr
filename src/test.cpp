// #include <Arduino.h>
// #include <SPI.h>
// #include "globals.h"  // for FLASH_* defines

// void setup() {
//     Serial.begin(115200);
//     delay(5000);

//     Serial.println("SPI JEDEC test...");

//     flashSPI.begin(FLASH_SCK, FLASH_MISO, FLASH_MOSI, FLASH_CS);

//     pinMode(FLASH_CS, OUTPUT);
//     digitalWrite(FLASH_CS, HIGH);

//     uint8_t jedec[3] = {0};

//     flashSPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(FLASH_CS, LOW);
//     flashSPI.transfer(0x9F); // JEDEC ID
//     jedec[0] = flashSPI.transfer(0x00);
//     jedec[1] = flashSPI.transfer(0x00);
//     jedec[2] = flashSPI.transfer(0x00);
//     digitalWrite(FLASH_CS, HIGH);
//     flashSPI.endTransaction();

//     Serial.printf("JEDEC ID: %02X %02X %02X\n", jedec[0], jedec[1], jedec[2]);
// }

// void loop() {
// }