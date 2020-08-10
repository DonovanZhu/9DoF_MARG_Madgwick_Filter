#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);

float accel[3];
float gyro[3];
int sample_times = 1;

float acc_cal[3] = {0.0, 0.0, 0.0};
float gyro_cal[3] = {0.0, 0.0, 0.0};

void read_sensors() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();
  
  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}

void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();

  // start communication with IMU 
  IMU.begin();
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);
}

void loop() {
  read_sensors();
  for (int i = 0; i < 3; i++) {
    acc_cal[i] += accel[i];
    gyro_cal[i] += gyro[i];
  }
  Serial.print(acc_cal[0] / sample_times, 7);
  Serial.print(" ");
  Serial.print(acc_cal[1] / sample_times, 7);
  Serial.print(" ");
  Serial.print(acc_cal[2] / sample_times, 7);
  Serial.print(" ");
  Serial.print(gyro_cal[0] / sample_times, 7);
  Serial.print(" ");
  Serial.print(gyro_cal[1] / sample_times, 7);
  Serial.print(" ");
  Serial.println(gyro_cal[2] / sample_times, 7);
  sample_times++;
  
}
