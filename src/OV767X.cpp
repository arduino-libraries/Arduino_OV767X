// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */

#include <Arduino.h>
#include <Wire.h>

#include "OV767X.h"

// if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 32))
#endif

#ifndef portInputRegister
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)
#endif

extern "C" {
  // defined in utility/ov7670.c:
  struct ov7670_fract {
    uint32_t numerator;
    uint32_t denominator;
  };

  void* ov7670_alloc();
  void ov7670_free(void*);

  int ov7670_reset(void*, uint32_t val);
  int ov7670_detect(void*);
  void ov7670_configure(void*, int devtype, int format, int wsize, int clock_speed, int pll_bypass, int pclk_hb_disable);
  int ov7670_s_power(void*, int on);
  int ov7675_set_framerate(void*, struct ov7670_fract *tpf);

  int ov7670_s_sat_hue(void*, int sat, int hue);
  int ov7670_s_brightness(void*, int value);
  int ov7670_s_contrast(void*, int value);
  int ov7670_s_hflip(void*, int value);
  int ov7670_s_vflip(void*, int value);
  int ov7670_s_gain(void*, int value);
  int ov7670_s_autogain(void*, int value);
  int ov7670_s_exp(void*, int value);
  int ov7670_s_autoexp(void*, int value);
  int ov7670_s_test_pattern(void*, int value);
};

const int OV760_D[8] = {
  OV7670_D0, OV7670_D1, OV7670_D2, OV7670_D3, OV7670_D4, OV7670_D5, OV7670_D6, OV7670_D7
};

OV767X::OV767X() :
  _ov7670(NULL),
  _saturation(128),
  _hue(0)
{
  setPins(OV7670_VSYNC, OV7670_HREF, OV7670_PLK, OV7670_XCLK, OV760_D);
}

OV767X::~OV767X()
{
  if (_ov7670) {
    ov7670_free(_ov7670);
  }
}

int OV767X::begin(int resolution, int format, int fps)
{
  switch (resolution) {
    case VGA:
      _width = 640;
      _height = 480;
      break;

    case CIF:
      _width = 352;
      _height = 240;
      break;

    case QVGA:
      _width = 320;
      _height = 240;
      break;

    case QCIF:
      _width = 176;
      _height = 144;
      break;

    default:
      return 0;
  }

  switch (format) {
    case YUV422:
    case RGB444:
    case RGB565:
      _bytesPerPixel = 2;
      break;

    default:
      return 0;
  }

  switch (fps) {
    case 1:
    case 5:
    case 10:
    case 15:
    case 30:
      break;

    default:
      return 0;
  }

  _ov7670 = ov7670_alloc();
  if (!_ov7670) {
    end();

    return 0;
  }

  pinMode(_vsyncPin, INPUT);
  pinMode(_hrefPin, INPUT);
  pinMode(_pclkPin, INPUT);
  pinMode(_xclkPin, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(_dPins[i], INPUT);
  }

  _vsyncPort = portInputRegister(digitalPinToPort(_vsyncPin));
  _vsyncMask = digitalPinToBitMask(_vsyncPin);
  _hrefPort = portInputRegister(digitalPinToPort(_hrefPin));
  _hrefMask = digitalPinToBitMask(_hrefPin);
  _pclkPort = portInputRegister(digitalPinToPort(_pclkPin));
  _pclkMask = digitalPinToBitMask(_pclkPin);

  for (int i = 0; i < 8; i++) {
    _dataPorts[i] = portInputRegister(digitalPinToPort(_dPins[i]));
    _dataMasks[i] = digitalPinToBitMask(_dPins[i]);
  }

  beginXClk();

  Wire.begin();

  delay(1000);

  if (ov7670_detect(_ov7670)) {
    end();

    return 0;
  }

  ov7670_configure(_ov7670, 0 /*OV7670 = 0, OV7675 = 1*/, format, resolution, 16 /* MHz */, 
                    0 /*pll bypass*/, 1 /* pclk_hb_disable */);

  if (ov7670_s_power(_ov7670, 1)) {
    end();

    return 0;
  }

  struct ov7670_fract tpf;

  tpf.numerator = 1;
  tpf.denominator = fps;

  ov7675_set_framerate(_ov7670, &tpf);

  return 1;
}

void OV767X::end()
{
  endXClk();

  pinMode(_xclkPin, INPUT);

  Wire.end();

  if (_ov7670) {
    ov7670_free(_ov7670);
  }
}

int OV767X::width() const
{
  return _width;
}

int OV767X::height() const
{
  return _height;
}

int OV767X::bitsPerPixel() const
{
  return _bytesPerPixel * 8;
}

int OV767X::bytesPerPixel() const
{
  return _bytesPerPixel;
}

void OV767X::readFrame(void* buffer)
{
  noInterrupts();

  uint8_t* b = (uint8_t*)buffer;
  int bytesPerRow = _width * _bytesPerPixel;

  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW

  for (int i = 0; i < _height; i++) {
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH

    for (int j = 0; j < bytesPerRow; j++) {
      while ((*_pclkPort & _pclkMask) != 0); // wait for LOW

      uint8_t in = 0;

      for (int k = 0; k < 8; k++) {
        bitWrite(in, k, (*_dataPorts[k] & _dataMasks[k]) != 0);
      }

      *b++ = in;

      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH
    }

    while ((*_hrefPort & _hrefMask) != 0); // wait for LOW
  }

  interrupts();
}

void OV767X::testPattern(int pattern)
{
  ov7670_s_test_pattern(_ov7670, pattern);
}

void OV767X::noTestPattern()
{
  ov7670_s_test_pattern(_ov7670, 0);
}

void OV767X::setSaturation(int saturation)
{
  _saturation = saturation;

  ov7670_s_sat_hue(_ov7670, _saturation, _hue);
}

void OV767X::setHue(int hue)
{
  _hue = hue;

   ov7670_s_sat_hue(_ov7670, _saturation, _hue);
}

void OV767X::setBrightness(int brightness)
{
  ov7670_s_brightness(_ov7670, brightness);
}

void OV767X::setContrast(int contrast)
{
  ov7670_s_contrast(_ov7670, contrast);
}

void OV767X::horizontalFlip()
{
  ov7670_s_hflip(_ov7670, 1);
}

void OV767X::noHorizontalFlip()
{
  ov7670_s_hflip(_ov7670, 0);
}

void OV767X::verticalFlip()
{
  ov7670_s_vflip(_ov7670, 1);
}

void OV767X::noVerticalFlip()
{
  ov7670_s_vflip(_ov7670, 0);
}

void OV767X::setGain(int gain)
{
  ov7670_s_gain(_ov7670, gain);
}

void OV767X::autoGain()
{
  ov7670_s_autogain(_ov7670, 1);
}

void OV767X::setExposure(int exposure)
{
  ov7670_s_exp(_ov7670, exposure);
}

void OV767X::autoExposure()
{
  ov7670_s_autoexp(_ov7670, 0 /* V4L2_EXPOSURE_AUTO */);
}

void OV767X::setPins(int vsync, int href, int pclk, int xclk, const int dpins[8])
{
  _vsyncPin = vsync;
  _hrefPin = href;
  _pclkPin = pclk;
  _xclkPin = xclk;

  memcpy(_dPins, dpins, sizeof(_dPins));
}

void OV767X::beginXClk()
{
  // Generates 16 MHz signal using I2S peripheral
  NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;

  NRF_I2S->PSEL.MCK = (digitalPinToPinName(_xclkPin) << I2S_PSEL_MCK_PIN_Pos);

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
}

void OV767X::endXClk()
{
  NRF_I2S->TASKS_STOP = 1;
}

OV767X Camera;
