#include <Arduino.h>
#include <Wire.h>

#include <Ewma.h> //https://github.com/jonnieZG/EWMA
#define MPU6050_ADRESS 0x68

static const double ACC_RATE = 16384.0;
static const double GYRO_RATE = 131.0;

double offsets[6];
unsigned long preInterval;

double roll = 0;
double pitch = 0;
double yaw = 0, yaw_ = 0;

//Ewma smoothing library
Ewma roll_filter(0.1);
Ewma pitch_filter(0.1);


double roll_lower_boarder = 1;
double roll_upper_boarder = -1;
double pitch_lower_boarder = 1;
double pitch_upper_boarder = -1;

/* helper functions */
double _max = 0, _min = 0;

/**
 * works like the built-in map function, but for double instead of long values
 * source: https://forum.arduino.cc/t/map-to-floating-point/3976
 * */
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * sets lower and upper boarders for roll and pitch, used to map values to certain range
 * @param val
 */
void set_roll_pitch_bounds(double roll_raw, double pitch_raw) {
    if (roll_raw > roll_upper_boarder) {
        roll_upper_boarder = roll_raw;
    }
    if (roll_raw < roll_lower_boarder) {
        roll_lower_boarder = roll_raw;
    }

    if (pitch_raw > pitch_upper_boarder) {
        pitch_upper_boarder = pitch_raw;
    }
    if (pitch_raw < pitch_lower_boarder) {
        pitch_lower_boarder = pitch_raw;
    }
}


/* MAIN */
/**
 * Get data from 6-axis sensor (MPU6050)
 * The Sensor has 7 registers containing: accelerationX, accelerationY, accelerationZ, temperature, gyroX, gyroY, gyroZ
 * @return Sensor data (Array size six with values: accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ)
 */
double *get_sensor_data() {
    //ToDo: smoothing: https://www.megunolink.com/articles/coding/3-methods-filter-noisy-arduino-measurements/
    static double _data[7];

    Wire.beginTransmission(MPU6050_ADRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false); // keep connection alive
    Wire.requestFrom(MPU6050_ADRESS, 7 * 2, true); // request a total of 7*2=14 registers

    //read registers from component (accelerationX, accelerationY, accelerationZ, temperature, gyroX, gyroY, gyroZ)
    for (int i = 0; i < 7; i++) {
        _data[i] = Wire.read() << 8 | Wire.read();
    }

    //acceleration
    for (int i = 0; i < 3; i++) {
        _data[i] = (_data[i] / ACC_RATE);
    }

    //gyro
    for (int i = 4; i < 7; i++) {
        _data[i] = _data[i] / GYRO_RATE;

    }

    return _data;
}


/**
 * calculate roll, pitch and yaw angles
 */
void *calculate_angles() {
    double *sensor_data = get_sensor_data();

    static const float boarder_roll_pitch = 2.0;  //upper_lower boarder_roll_pitch for roll and pitch
    static const double boarder_yaw = 4.5;  //upper_lower boarder_roll_pitch for roll and pitch

    static const double map_to = 1.0;
    double roll_raw = sensor_data[0] - offsets[0];
    double pitch_raw = sensor_data[1] - offsets[1];

    set_roll_pitch_bounds(roll_raw, pitch_raw);


    //roll
    roll_raw = roll_filter.filter(roll_raw);
    roll = mapDouble(roll_raw, roll_lower_boarder, roll_upper_boarder, -map_to, map_to);
    roll = constrain(roll, -map_to, map_to);

    //pitch
    pitch_raw = pitch_filter.filter(pitch_raw);
    pitch = mapDouble(pitch_raw, pitch_lower_boarder, pitch_upper_boarder, -map_to, map_to);
    pitch = constrain(pitch, -map_to, map_to);


    if (abs(roll) < 0.1) {
        roll = 0;
    }
    if (abs(pitch) < 0.1) {
        pitch = 0;
    }

    //yaw
    yaw = 0;
    unsigned long Tnew = millis();
    double dt = (Tnew - preInterval) * 0.001;
    preInterval = Tnew;

    double add_to_yaw = sensor_data[6] * dt;

    if (abs(add_to_yaw) > 0.02 && abs(yaw_ + add_to_yaw) < 1) {
        yaw_ += add_to_yaw;
    }

    yaw = mapDouble(yaw_, -boarder_yaw, boarder_yaw, -map_to, map_to);
    yaw = constrain(yaw, -map_to, map_to);

    bool print;
    print = false;
    print = true;


    if (print) {
        Serial.print("roll:");
        Serial.print(roll);
        Serial.print(" \t");
        Serial.print("pitch:");
        Serial.print(pitch);
        Serial.print(" \t");
        Serial.print("yaw:");
        Serial.print(yaw);
        Serial.println(" \t");
    }

}

/**
 * Calculating sensor offset.
 * Values are measured 200 times with a delay of 10ms
 * @return Array with average offset values for acceleration and gyro values
 */
void calculate_offsets() {
    Serial.println("Calculating offsets");
    static const int REPS = 200;
    static const int DELAY = 10;
    static const int OFFSET_MULTIPLIER = 10;

    double _offsets[6];

    for (int i = 0; i < REPS; i++) {
        double *data = get_sensor_data();

        _offsets[0] += data[0];
        _offsets[1] += data[1];
        _offsets[2] += data[2];

        _offsets[3] += data[4];
        _offsets[4] += data[5];
        _offsets[5] += data[6];
        delay(DELAY);
    }

    for (int i = 0; i < 6; i++) {
        offsets[i] = _offsets[i] / REPS;

        if (offsets[i] > 2147483648.0) { offsets[i] = 0; }
        if (offsets[i] < -2147483648.0) { offsets[i] = 0; }
        if (isnan(offsets[i])) { offsets[i] = 0; }
    }

}


void setup() {
    Wire.begin();
    Wire.beginTransmission(MPU6050_ADRESS); // Begins a transmission to the I2C slave (GY-521 board)
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    Serial.begin(9600);
    Serial.flush();

    calculate_offsets();
}


void loop() {
    preInterval = millis();
    calculate_angles();

    delay(10);
}