#include <Wire.h>
#include <SPI.h>

// Define SPI pins
const int SPI_MISO = 17; // MISO (Master In Slave Out)
const int SPI_MOSI = 16; // MOSI (Master Out Slave In)
const int SPI_SCK = 5;   // SCK (Serial Clock)
const int SPI_CS = 23;   // CS (Chip Select)

// Define I2C pins
const int I2C_SDA = 21; // SDA (Serial Data)
const int I2C_SCL = 22; // SCL (Serial Clock)

void setup() {
  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  pinMode(SPI_CS, OUTPUT);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Other setup code
}

void loop() {
  // Your SPI and I2C communication code here
}
