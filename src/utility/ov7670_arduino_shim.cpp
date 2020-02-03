// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */
#include <Arduino.h>
#include <Wire.h>

#ifndef OV760_DEBUG
// #define OV760_DEBUG
#endif

extern "C" {
  void msleep(unsigned long ms)
  {
    delay(ms);
  }

  int arduino_i2c_read(unsigned short address, unsigned char reg, unsigned char *value)
  {
#ifdef OV760_DEBUG
    Serial.print("arduino_i2c_read: address = 0x");
    Serial.print(address, HEX);
    Serial.print(", reg = 0x");
    Serial.print(reg, HEX);
#endif

    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
#ifdef OV760_DEBUG
    Serial.println();
#endif
      return -1;
    }

    if (Wire.requestFrom(address, 1) != 1) {
#ifdef OV760_DEBUG
      Serial.println();
#endif
      return -1;
    }

    *value = Wire.read();

#ifdef OV760_DEBUG
    Serial.print(", value = 0x");
    Serial.println(*value, HEX);
#endif

    return 0;
  }

  int arduino_i2c_write(unsigned short address, unsigned char reg, unsigned char value)
  {
#ifdef OV760_DEBUG
    Serial.print("arduino_i2c_write: address = 0x");
    Serial.print(address, HEX);
    Serial.print(", reg = 0x");
    Serial.print(reg, HEX);
    Serial.print(", value = 0x");
    Serial.println(value, HEX);
#endif

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
   
    if (Wire.endTransmission() != 0) {
      return -1;
    }

    return 0;
  }
};
