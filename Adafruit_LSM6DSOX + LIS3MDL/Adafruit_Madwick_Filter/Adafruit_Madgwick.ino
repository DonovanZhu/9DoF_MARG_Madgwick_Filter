#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "MadgwickAHRS.h"
#include <Wire.h>

Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mage;
sensors_event_t temp;


//-0.0265399 -0.3667369 10.0539670 0.0139412 0.0008121 -0.0068567

#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET 0.0139412
#define GYRO_Y_OFFSET 0.0008121
#define GYRO_Z_OFFSET -0.0068567

#define ACCEL_X_OFFSET -0.0265399
#define ACCEL_Y_OFFSET -0.3667369
#define ACCEL_Z_OFFSET 10.0539670


const float magn_ellipsoid_center[3] = {-36.7351, -9.37435, -2.96601};
const float magn_ellipsoid_transform[3][3] = {{0.915205, 0.0247975, -0.0215413}, {0.0247975, 0.983636, -0.0171572}, {-0.0215413, -0.0171572, 0.934144}};
// Sensor variables
float acc[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float mag[3];
float mag_tmp[3];
float gyr[3];

float time_now;
float time_former;
float deltat;

double rotation_matrix[3][3] = {0};

// Set structs for converting result from Quaternion to Euler angles
struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

Quaternion qua;
EulerAngles eul;

void read_sensors() {
  sox.getEvent(&accel, &gyro, &temp);
  lis.getEvent(&mage);
  acc[0] = accel.acceleration.x;
  acc[1] = accel.acceleration.y;
  acc[2] = accel.acceleration.z;

  mag[0] = mage.magnetic.x;
  mag[1] = mage.magnetic.y;
  mag[2] = mage.magnetic.z;

  gyr[0] = gyro.gyro.x;
  gyr[1] = gyro.gyro.y;
  gyr[2] = gyro.gyro.z;
}

void compensate_sensor_errors() {
    // Compensate accelerometer error
    acc[0] -= ACCEL_X_OFFSET;
    acc[1] -= ACCEL_Y_OFFSET;
    acc[2] -= ACCEL_Z_OFFSET - GRAVITY;

    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      mag_tmp[i] = mag[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, mag_tmp, mag);

    // Compensate gyroscope error
    gyr[0] -= GYRO_X_OFFSET;
    gyr[1] -= GYRO_Y_OFFSET;
    gyr[2] -= GYRO_Z_OFFSET;
}

/* This file is part of the Razor AHRS Firmware */
// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

void sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll_e = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch_e = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch_e = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw_e = atan2(siny_cosp, cosy_cosp);

    return angles;
}


void quaternion_to_matrix()
{
  rotation_matrix[0][0] = 1 - 2 * qua.y * qua.y - 2 * qua.z * qua.z;
  rotation_matrix[0][1] = 2 * qua.x * qua.y - 2 * qua.z * qua.w;
  rotation_matrix[0][2] = 2 * qua.x * qua.z + 2 * qua.y * qua.w;
  rotation_matrix[1][0] = 2 * qua.x * qua.y + 2 * qua.z * qua.w;
  rotation_matrix[1][1] = 1 - 2 * qua.x * qua.x - 2 * qua.z * qua.z;
  rotation_matrix[1][2] = 2 * qua.y * qua.z - 2 * qua.x * qua.w;
  rotation_matrix[2][0] = 2 * qua.x * qua.z - 2 * qua.y * qua.w;
  rotation_matrix[2][1] = 2 * qua.y * qua.z + 2 * qua.x * qua.w;
  rotation_matrix[2][2] = 1 - 2 * qua.x * qua.x - 2 * qua.y * qua.y;

  
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();
  //Wire.begin();
  Wire.setClock(4000000);
  sox.begin_I2C();

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );

  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);

  sox.setGyroDataRate(LSM6DS_RATE_52_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire

  lis.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);

  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis.setDataRate(LIS3MDL_DATARATE_40_HZ);

  lis.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, // enable z axis
                      true, // polarity
                      false, // don't latch
                      true); // enabled!
  time_former = micros();
}

void loop() {
  read_sensors();

  compensate_sensor_errors();

  time_now = micros();
  deltat = (float)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  MadgwickQuaternionUpdate(acc[0], acc[1], acc[2],
                           gyr[0], gyr[1], gyr[2],
                           mag[0], mag[1], mag[2], deltat);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];

  quaternion_to_matrix();

  
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8; // 0.8 this used for compensating the angle between magenatic north and geographic north
  if (eul.yaw_e > PI) {
    eul.yaw_e -= 2 * PI;
  }
  /*
  Serial.print(eul.roll_e * 180.0 / PI);
  Serial.print(" ");
  Serial.print(eul.pitch_e * 180.0 / PI);
  Serial.print(" ");
  Serial.print(eul.yaw_e * 180.0 / PI);
  Serial.println(" ");
  */
  // Serial.println(deltat,7);
  // sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  delayMicroseconds(1000);
}
