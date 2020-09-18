# 9 Degree of Freedom MARG Madgwick Filter Attitude Estimation

## Project Description
This project implements attitude estimation by using [Madgwick filter](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/), which is a low computational cost and high accuracy data fusion algorithm, based on a 9 degree of freedom MARG(magnetic angular rate and gravity, i.e., IMU + magnetometer). In this project, the algorithm is implemented in two type of MARG: [MPU9250](https://www.amazon.com/HiLetgo-Gyroscope-Acceleration-Accelerator-Magnetometer/dp/B01I1J0Z7Y/ref=sr_1_4?dchild=1&keywords=MPU9250&qid=1597109421&sr=8-4) and 
[Adafruit LSM6DSOX + LIS3MDL](https://www.adafruit.com/product/4517).
MPU9250 can also be replaced by [MPU9255](https://www.amazon.com/UCTRONICS-MPU-9255-Compass-Accelerometer-Gyroscope/dp/B01DIGRR8U/ref=sr_1_3?dchild=1&keywords=MPU9255&qid=1597109290&sr=8-3) since they use same library in Arduino. A [Teensy4.0](https://www.pjrc.com/teensy-4-0/), which is an Arduino like microcontroller, is used to receive and process the data from MARG, and sent results of attitude estimation as Euler angle to PC.

**To view the performance, please click the following image.**

[![Alt text](https://img.youtube.com/vi/iOwcov_5z3c/0.jpg)](https://www.youtube.com/watch?v=iOwcov_5z3c)

## Hardware Description

To set a simple attitude estimation system, only to unit is needed: a microcontroller and a 9 DoF MARG. In this project, Teensy4.0 is used as microcontroller to receive and process data from MARG.

### Setting up the hardware
Connection between Teensy4.0 and MPU9250 or Adafruit LSM6DSOX + LIS3MDL is as follows. Teensy connects PC through micro-USB port to transfer data.
```
 TEENSY4.0 <--> MARG
 
 3.3V      <--> VCC
 GND       <--> GND
 SCL       <--> SCL
 SDA       <--> SDA
```
<img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Teensy_MPU9250_Connection.jpg" height="500"> <img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Teensy_Adafruit_Connection.jpg" height="500">

### microcontroller
[Teensy](https://www.pjrc.com/teensy/) is a complete USB-based microcontroller development system. It is compatible with Arduino Software & Libraries. To use it, Arduino software and Teensyduino package is required. The official installation instruction is [here](https://www.pjrc.com/teensy/td_download.html).



## Setting up the software

### Calibration
MPU9250 is a 9 DoF MARG. Accelerometer, gyroscope and magnetometer are impelemented on MPU9250 to sense accleration, rotation speed and magnetic field along the three axises of the coordinate on itself. To successfully estimate the eular angle, calibration of all 9 data is required.
