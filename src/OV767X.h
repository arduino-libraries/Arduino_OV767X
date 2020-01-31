// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX7670 library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */

#ifndef _OV767X_H_
#define _OV767X_H_

#include <Arduino.h>

#define OV7670_VSYNC 8
#define OV7670_HREF  10
#define OV7670_PLK   12
#define OV7670_XCLK  9
#define OV7670_D0    0
#define OV7670_D1    1
#define OV7670_D2    2
#define OV7670_D3    3
#define OV7670_D4    4
#define OV7670_D5    5
#define OV7670_D6    6
#define OV7670_D7    7

enum
{
  YUV422 = 0,
  RGB444 = 1,
  RGB565 = 2,
//  SBGGR8 = 3
};

enum
{
  VGA = 0,
  CIF = 1,
  QVGA = 2,
  QCIF = 3
};

class OV767X
{
public:
  OV767X();
  virtual ~OV767X();

  int begin(int resolution, int format, int fps);
  void end();

  int width() const;
  int height() const;
  int bitsPerPixel() const;

  void readFrame(void* buffer);

  void testPattern();
  void noTestPattern();

  void setPins(int vsync, int hrefPin, int pclkPin, int xclk, const int dpins[8]);

private:
  void beginXClk();

private:
  int _vsyncPin;
  int _hrefPin;
  int _pclkPin;
  int _xclkPin;
  int _dPins[8];

  int _width;
  int _height;
  int _bitsPerPixel;

  void* _ov7670;

  volatile uint32_t* _vsyncPort;
  volatile uint32_t _vsyncMask;
  volatile uint32_t* _hrefPort;
  volatile uint32_t _hrefMask;
  volatile uint32_t* _pclkPort;
  volatile uint32_t _pclkMask;
  volatile uint32_t* _dataPorts[8];
  uint32_t _dataMasks[8];
};

extern OV767X Camera;

#endif