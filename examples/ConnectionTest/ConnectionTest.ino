/*
  OV767X - ConnectionTest.ino

  Test that the connection between your Arduino and Camera is able to transfer data correctly at the given speed
  
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
#include <Arduino_CRC32.h>

int bytesPerFrame;
int errors = 0;
int count = 0;
int bestTime = 100000;
int worstTime = 0;
int delayTime = 300;


long timer = 0;
Arduino_CRC32 crc32;

const bool error_checking = true;

byte data[176 * 144 * 2]; // QCIF at 2 bytes per pixel

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!Camera.begin(QCIF, RGB565, 5)) {
    Serial.println("Failed to initialize camera!");
    while (1);
  }

  bytesPerFrame = Camera.width() * Camera.height() * Camera.bytesPerPixel();

  // Enable the test pattern so we have a fixed image to run a checksum against
  Camera.testPattern();
}

void loop() {

  // sliding delay window to try different start times wrt camera VSYNC
  if (delayTime>0) {delayTime=delayTime-10;}
  delay(delayTime);
  
  // benchmarking
  timer = millis();
  Camera.readFrame(data);
  timer = millis() - timer;
  // Check if it is a best case or worse case time
  bestTime = min(timer, bestTime);
  worstTime = max(timer, worstTime);

  // Test against known checksum values (minor pixel variations at the start but were visually confirmed to be a good test pattern)
  uint32_t const crc32_res = crc32.calc(data, bytesPerFrame);
  if (crc32_res != 0x15AB2939 && crc32_res != 0xD3EC95E && crc32_res != 0xB9C43ED9) {
    errors++;
  };

  count++;

  Serial.print(" errors:");
  Serial.print(errors);
  Serial.print("/");
  Serial.print(count);
  Serial.print(" best:");
  Serial.print(bestTime);
  Serial.print("ms worst:");
  Serial.print(worstTime);
  Serial.println("ms");
  
}
