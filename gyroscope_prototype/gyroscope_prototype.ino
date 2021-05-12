#include <Wire.h>

#define MPU6050_ADRESS 0x68
#define RAD_2_DEG 57.2957786

float ACC_RATE = 16384.0;
float GYRO_RATE = 131.0;

float * offsets;
unsigned long preInterval;

float angleX;
float angleY;
float angleZ = 0;

float * get_data() {
  static float _data[7];

  Wire.beginTransmission(MPU6050_ADRESS);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU6050_ADRESS, 7 * 2, true); // request a total of 7*2=14 registers

  for (int i = 0; i < 7; i++) {
    _data[i] = Wire.read() << 8 | Wire.read();
    //Serial.print("data="); Serial.print(_data[i]); Serial.print("\t ");
  }

  for (int i = 0; i < 3; i++) {
    _data[i] = (_data[i] / ACC_RATE) - (offsets[i] / ACC_RATE);
  }
  for (int i = 4; i < 7; i++) {
    _data[i] = _data[i] / GYRO_RATE;
  }

  /*
    for (int i = 0; i < 7; i++) {
      Serial.print("data="); Serial.print(_data[i]); Serial.print("\t ");
    }
    Serial.println();
  */

  return _data;
}


float * calculate_angles(float offset_acc_x, float offset_acc_y, float offset_acc_z, float offset_gyro_x, float offset_gyro_y, float offset_gyro_z ) {
  float acc_rate = 16384.0;
  float gyro_rate = 131.0;

  float _angles[3];

  float * data = get_data();

  data[0] = map(data[0] * 100, -100, -90.0, 100, 90.0);

  if (data[0] > -5 && data[0] < 5 ) {
    data[0] = 0;
  }
  data[0] = constrain(data[0], -90, 90);


  data[1] = map(data[1] * 100, -900, -90.0, 900, 90.0);

  if (data[1] > -6 && data[1] < 6 ) {
    data[1] = 0;
  }

  data[1] = constrain(data[1], -90, 90);



  float realAngleZ = 0;
  float gyro_z = data[6];
  unsigned long Tnew = millis();
  float dt = (Tnew - preInterval) * 1e-3;
  preInterval = Tnew;

  float add_to_angle =  gyro_z * dt;

  if ( abs(add_to_angle) > 0.02) {
    angleZ += add_to_angle;
  }

  realAngleZ = map(angleZ * 10, -45, -90.0, 45, 90.0);
  realAngleZ = constrain(realAngleZ, -90, 90);

  Serial.print("x:"); Serial.print( data[0]); Serial.print(" \t");
  Serial.print("y:"); Serial.print( data[1]); Serial.print(" \t");
  Serial.print("z:"); Serial.print(realAngleZ); Serial.println(" \t");

  return _angles;

}



float * calculate_offsets() {
  Serial.println("Calculating offsets");
  static const int REPS = 200;
  static const int DELAY = 10;

  float _offsets[6];

  for (int i = 0; i < REPS;  i++) {
    float *data = get_data();

    _offsets[0] += data[0];
    _offsets[1] += data[1];
    _offsets[2] += data[2];

    _offsets[3] += data[4];
    _offsets[4] += data[5];
    _offsets[5] += data[6];
    //Serial.print(i); Serial.print("/"); Serial.println(REPS);
    delay(5);
  }

  for (int i = 0; i < 6;  i++) {
    _offsets[i] = _offsets[i] / REPS;
  }


  //Serial.println("Calculated offsets");
  for (int i = 0; i < 6;  i++) {
    //Serial.print("_offsets["); Serial.print(i); Serial.print("]: "); Serial.print(_offsets[i]);  Serial.print("\t ");
  }

  //Serial.println();
  return _offsets;

}


void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADRESS); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(9600);
  Serial.flush();

  calculate_offsets();
}


void loop()
{
  preInterval = millis();
  float * angles = calculate_angles(offsets[0], offsets[1], offsets[3], offsets[4], offsets[5], offsets[6]);

  Serial.println();
  delay(10);
}
