/*
  OV767X - Camera Test Pattern

  This sketch enables the test pattern mode, then reads a frame from
  the OmniVision OV7670 camera and prints the data to the 
  Serial Monitor as a hex string.

  The website https://rawpixels.net - can be used the visualize the data:
    width: 176
    height: 144
    RGB565
    Little Endian

  Circuit:
    - Arduino Nano 33 BLE board
    - OV7670 camera module:
      - 3.3 connected to 3.3
      - GND connected GND
      - SIOC connected to A5
      - SIOD connected to A4
      - VSYNC connected to 8
      - HREF connected to A1
      - PCLK connected to A0
      - XCLK connected to 9
      - D7 connected to 4
      - D6 connected to 6
      - D5 connected to 5
      - D4 connected to 3
      - D3 connected to 2
      - D2 connected to 0 / RX
      - D1 connected to 1 / TX
      - D0 connected to 10

  This example code is in the public domain.
*/

#include <Arduino_OV767X.h>

unsigned short pixels[176 * 144]; // QCIF: 176x144 X 2 bytes per pixel (RGB565)

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("OV767X Test Pattern");
  Serial.println();

  if (!Camera.begin(QCIF, RGB565, 1)) {
    Serial.println("Failed to initialize camera!");
    while (1);
  }

  Serial.println("Camera settings:");
  Serial.print("\twidth = ");
  Serial.println(Camera.width());
  Serial.print("\theight = ");
  Serial.println(Camera.height());
  Serial.print("\tbits per pixel = ");
  Serial.println(Camera.bitsPerPixel());
  Serial.println();

  Serial.println("Enabling test pattern mode");
  Serial.println();
  Camera.testPattern();

  Serial.println("Reading frame");
  Serial.println();
  Camera.readFrame(pixels);

  int numPixels = Camera.width() * Camera.height();

  for (int i = 0; i < numPixels; i++) {
    unsigned short p = pixels[i];

    if (p < 0x1000) {
      Serial.print('0');
    }

    if (p < 0x0100) {
      Serial.print('0');
    }

    if (p < 0x0010) {
      Serial.print('0');
    }

    Serial.print(p, HEX);
  }

  Serial.println();
}

void loop() {
  // do nothing
}
