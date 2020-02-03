/*
  OV767X - Camera Text Pattern

  This enables reads the test pattern from the OV7670 camera
  and prints the data to the Serial Monitor as a hex string.

  The website https://rawpixels.net - can be used the visualize the data:
    width: 176
    height: 144
    RGB565
    Big Endian

  Circuit:
    - Arduino Nano 33 BLE board
    - OV7670 camera module:
      - 3.3 connected to 3.3
      - GND connected GND
      - SIOC connected to A5
      - SIOD connected to A4
      - VSYNC connected to 8
      - PCLK connected to 12
      - XCLK connected to 9
      - D7 connected to 7
      - D6 connected to 6
      - D5 connected to 5
      - D4 connected to 4
      - D3 connected to 3
      - D2 connected to 2
      - D1 connected to 1 / RX
      - D0 connected to 0 / TX

  This example code is in the public domain.
*/

#include <Arduino_OV767X.h>

byte frameBytes[176 * 144 * 2]; // QCIF: 176x144 X 2 bytes per pixel (RGB565)

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

//  Serial.println("Enabling test pattern mode");
//  Serial.println();
//  Camera.testPattern();

delay(5000);

  Serial.println("Reading frame");
  Serial.println();
  Camera.readFrame(frameBytes);

  for (unsigned int i = 0; i < sizeof(frameBytes); i++) {
    byte b = frameBytes[i];

    if (b < 16) {
      Serial.print('0');
    }
    Serial.print(b, HEX);
  }
  Serial.println();
}

void loop() {
  // do nothing
}
