#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

bool shtp_init(TwoWire *i2cBus, uint8_t address, uint8_t intPin);
bool shtp_init_spi(SPIClass *spiBus, uint8_t csPin, uint8_t intPin, uint32_t spiSpeed);
