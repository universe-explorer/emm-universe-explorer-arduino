/**
 * Potentiometer
 * 
 * Info about the protocol structure is in our first presentation
 */

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
