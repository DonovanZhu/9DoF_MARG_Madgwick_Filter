#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>


Adafruit_LSM6DS sox;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

double acc[3] = {0.0, 0.0, 0.0};
double gyr[3] = {0.0, 0.0, 0.0};

double acc_cal[3] = {0.0, 0.0, 0.0};
double gyro_cal[3] = {0.0, 0.0, 0.0};

int sample_times = 1;

void read_sensors() {
  sox.getEvent(&accel, &gyro, &temp);

  acc[0] = accel.acceleration.x;
  acc[1] = accel.acceleration.y;
  acc[2] = accel.acceleration.z;
  
  gyr[0] = gyro.gyro.x;
  gyr[1] = gyro.gyro.y;
  gyr[2] = gyro.gyro.z;
}


void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();
  Wire.begin();
  Wire.setClock(4000000);
  sox.begin_I2C();

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);

  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );

  sox.setAccelDataRate(LSM6DS_RATE_52_HZ);

  sox.setGyroDataRate(LSM6DS_RATE_52_HZ);

}

void loop() {
  read_sensors();
  for (int i = 0; i < 3; i++) {
    acc_cal[i] += acc[i];
    gyro_cal[i] += gyr[i];
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
