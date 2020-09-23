#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "MadgwickAHRS.h"

Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
sensors_event_t mage;

const double magn_ellipsoid_center[3] = {0.262689, -6.89484, 4.0776};
const double magn_ellipsoid_transform[3][3] = {{0.899993, 0.0341615, -0.000181209}, {0.0341615, 0.988324, -0.000514259}, { -0.000181209, -0.000514259, 0.952566}};

double  acc[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
double  mag[3];
double  mag_tmp[3];
double  gyr[3];

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


void setup() {
  time_former = micros();

  sox.begin_I2C();
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire
  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, true, false, true); // enabled!

  // Open Serial port in speed 115200 Baudrate
  Serial.begin(USB_UART_SPEED);
}

void loop() {

  read_sensors();
  
  compensate_sensor_errors();
  
  time_now = (double)micros();
  deltat = (time_now - time_former) / 1000000.0;
  time_former = time_now;
  
  MadgwickQuaternionUpdate(acc[0], acc[1], acc[2],
                         gyr[0], gyr[1], gyr[2],
                         mag[0], mag[1], mag[2], deltat);
  
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
  eul = ToEulerAngles(qua);

  sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  delayMicroseconds(1000);
}
