/**
 * Usage of our protocol in a simple example
 * 
 * Info about the protocol structure is in our first presentation
 */

const byte startByte = 0xBE;
const byte bufferLength = 2;
byte dataBuffer[bufferLength];


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

  Serial.write(startByte);
  Serial.write(bufferLength);

  
  dataBuffer[0] = 0x00; // speed byte
  dataBuffer[1] = 0xFF; // speed value

  Serial.write(dataBuffer, bufferLength);

  //uint8_t crc = gencrc(dataBuffer, bufferLength);
  Serial.write(gencrc(dataBuffer, bufferLength));

  delay(1);
  
}
