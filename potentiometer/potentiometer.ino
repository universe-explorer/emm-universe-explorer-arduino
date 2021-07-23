/**
 * Potentiometer
 * 
 * Info about the protocol structure is in our first presentation
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


const byte startByte = 0xBE;
const byte bufferLength = 2;

const int potentiometerPin = A0;

uint8_t gencrc(uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x15);
            else
                crc <<= 1;
        }
    }
    return crc;
}



void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.display();

}

void loop() {

  byte dataBuffer[bufferLength];
  
  Serial.write(startByte);
  Serial.write(bufferLength);
  
  
  dataBuffer[0] = 0x00; // speed byte

  int read = analogRead(potentiometerPin);
  dataBuffer[1] = map(read, 0, 1023, 0, 255); // speed value
  //Serial.println(dataBuffer[1]);

  Serial.write(dataBuffer, bufferLength);

  uint8_t crc = gencrc(dataBuffer, bufferLength);
  Serial.write(gencrc(dataBuffer, bufferLength));

  delay(1);
  
}
