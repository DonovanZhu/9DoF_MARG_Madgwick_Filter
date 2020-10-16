#include <Adafruit_LIS3MDL.h>
#include <Wire.h>

Adafruit_LIS3MDL lis;
sensors_event_t mage;

float mag[3];

void read_sensors() {
  lis.getEvent(&mage);
  mag[0] = mage.magnetic.x;
  mag[1] = mage.magnetic.y;
  mag[2] = mage.magnetic.z;
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

void setup() {
  Serial.begin(1000000);
  while (!Serial) yield();
  //Wire.begin();
  Wire.setClock(4000000);

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
}

void loop() {
  read_sensors();
  delayMicroseconds(1000);
  sendToPC(&mag[0], &mag[1], &mag[2]);
}
