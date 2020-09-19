# 9 Degree of Freedom MARG Madgwick Filter Attitude Estimation

## Project Description
This project implements attitude estimation by using [Madgwick filter](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/), which is a low computational cost and high accuracy data fusion algorithm, based on a 9 degree of freedom MARG(magnetic angular rate and gravity, i.e., IMU + magnetometer). In this project, the algorithm is implemented in two type of MARG: [MPU9250](https://www.amazon.com/HiLetgo-Gyroscope-Acceleration-Accelerator-Magnetometer/dp/B01I1J0Z7Y/ref=sr_1_4?dchild=1&keywords=MPU9250&qid=1597109421&sr=8-4) and 
[Adafruit LSM6DSOX + LIS3MDL](https://www.adafruit.com/product/4517).
MPU9250 can also be replaced by [MPU9255](https://www.amazon.com/UCTRONICS-MPU-9255-Compass-Accelerometer-Gyroscope/dp/B01DIGRR8U/ref=sr_1_3?dchild=1&keywords=MPU9255&qid=1597109290&sr=8-3) since they use same library in Arduino. A [Teensy4.0](https://www.pjrc.com/teensy-4-0/), which is an Arduino like microcontroller, is used to receive and process the data from MARG, and sent results of attitude estimation as Euler angle to PC.

**To view the performance, please click the following image.**

[![Alt text](https://img.youtube.com/vi/iOwcov_5z3c/0.jpg)](https://www.youtube.com/watch?v=iOwcov_5z3c)

## Hardware Description

To set a simple attitude estimation system, only two units are needed: a microcontroller and a 9 DoF MARG. In this project, Teensy4.0 is used as microcontroller to receive and process data from MARG.

### Setting up the hardware
Connection between Teensy4.0 and MPU9250 or Adafruit LSM6DSOX + LIS3MDL is as follows. Teensy connects PC through micro-USB port to transfer data.
```
 TEENSY4.0 <--> MARG
 
 3.3V      <--> VCC
 GND       <--> GND
 SCL       <--> SCL
 SDA       <--> SDA
```

MARG is powered by Teensy from its 3.3V and GND pins. The MARG used in this project uses I2C interface to send data. I2C is a stable and relatively high-speed interface with only 2 wire. To check the pins of Teensy4.0, a pinout diagram can be found on [offical website](https://www.pjrc.com/store/teensy40.html).

<img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Teensy_MPU9250_Connection.jpg" height="500"> <img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Teensy_Adafruit_Connection.jpg" height="500">

### Teensy4.0
[Teensy](https://www.pjrc.com/teensy/) is a complete USB-based microcontroller development system. It is compatible with Arduino Software & Libraries. To use it, Arduino software and Teensyduino package is required. The official installation instruction is [here](https://www.pjrc.com/teensy/td_download.html). Teensy is a powerful and small size microcontroller. It features an ARM Cortex-M7 processor at 600 MHz, with a NXP iMXRT1062 chip, one of the fastest microcontroller available today.

### MARG
MARG(magnetic angular rate and gravity), also called 9 Degree of Freedom IMU, can sense 3 types data: acceleration, rotation speed and magnetic field. 9 Dof means it provides these 3 types data along X/Y/Z axis of IMU, which are 9 numbers. Different IMU has different coordinate. Producers always paint that on the board, or can be found on official datasheet.

## Algorithm Description
Madgwick filter is a data fusion algorithm for alttitude estimation. To estimation the rotation angle along each axis of base coordinate, only use one type of data is not enough. For example, a simple way to calculate rotation angle is doing intergration of rotation speed provided by the gyroscope in IMU. However, the raw data is not stable, and the worst is the bias. When doing integration, bias will also be included in integration result. As a result, the rotation angle will keep rotating even the user does not touch it. This phenomenon is so called "drift".

Data fusion algorithm is a good way to hangle drift and improving the estimation accuracy. It fuse all the data from IMU, each data provides useful information of IMU, and reduce the error produced by other type of data. 

At present, many data fusion algorithms are able to estimate accurate attitude, e.g. [Mahony filter](https://ieeexplore.ieee.org/document/4608934), [Extanded Kalmen filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) and Madgwick filter. In these algorithm, EFK provides high accuracy, but it consumes a lot computing power. Madgwick filter is accurate enough for most useage and it needs less computing resources.

To understand Madgwick filter, It is highly recommanded to read the [orignal paper](https://x-io.co.uk/res/doc/madgwick_internal_report.pdf).

## Setting up the software

### Calibration
MPU9250 is a 9 DoF MARG. Accelerometer, gyroscope and magnetometer are impelemented on MPU9250 to sense accleration, rotation speed and magnetic field along the three axises of the coordinate on itself. To successfully estimate the eular angle, calibration of all 9 data is required.
