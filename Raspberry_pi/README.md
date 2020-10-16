# 9 Degree of Freedom MARG Madgwick Filter Attitude Estimation

## Project Description
This project implements attitude estimation by using [Madgwick filter](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/), which is a low computational cost and high accuracy data fusion algorithm, based on a 9 degree of freedom MARG(magnetic angular rate and gravity, i.e., IMU + magnetometer). In this project, the algorithm is implemented in [Adafruit LSM6DSOX + LIS3MDL](https://www.adafruit.com/product/4517). A [Raspberry Pi 4 B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) is used to receive and process the data from MARG to estimate the attitude of IMU.

**To view the performance, please click the following image.**

[![Alt text](https://img.youtube.com/vi/iOwcov_5z3c/0.jpg)](https://www.youtube.com/watch?v=iOwcov_5z3c)

## Hardware Description

To set a simple attitude estimation system, only two units are needed: a raspberry pi and a 9 DoF MARG.

### Setting up the Hardware
Connection between Raspberry pi 4B and Adafruit LSM6DSOX + LIS3MDL is as follows.
```
 RPi  <--> MARG
 
 3.3V <--> VCC
 GND  <--> GND
 SCL  <--> SCL
 SDA  <--> SDA
```

MARG is powered by Teensy from its 3.3V and GND pins. The MARG used in this project uses I2C interface to send data. I2C is a stable and relatively high-speed interface with only 2 wire. To check the pins of Teensy4.0, a pinout diagram can be found on [official website](https://www.pjrc.com/store/teensy40.html).

<img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Raspberry_pi/RPi_Adafruit_Connection.jpg" height="500">

### MARG
MARG(magnetic angular rate and gravity), also called 9 Degree of Freedom IMU, can sense 3 types data: acceleration, rotation speed and magnetic field. 9 Dof means it provides these 3 types data along X/Y/Z axis of IMU, which are 9 numbers. Different IMU has different coordinate. Producers always paint that on the board, or can be found on official datasheet.

## Algorithm Description
Madgwick filter is a data fusion algorithm for altitude estimation. To estimation the rotation angle along each axis of base coordinate, only use one type of data is not enough. For example, a simple way to calculate rotation angle is doing integration of rotation speed provided by the gyroscope in IMU. However, the raw data is not stable, and the worst is the bias. When doing integration, bias will also be included in integration result. As a result, the rotation angle will keep rotating even the user does not touch it. This phenomenon is so called "drift".

Data fusion algorithm is a good way to handle drift and improving the estimation accuracy. It fuse all the data from IMU, each data provides useful information of IMU, and reduce the error produced by other type of data. 

At present, many data fusion algorithms are able to estimate accurate attitude, e.g. [Mahony filter](https://ieeexplore.ieee.org/document/4608934), [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) and Madgwick filter. In these algorithms, EFK provides high accuracy, but it consumes a lot computing power. Madgwick filter is accurate enough for most usage and it needs less computing resources.

To understand Madgwick filter, It is highly recommended to read the [original paper](https://x-io.co.uk/res/doc/madgwick_internal_report.pdf).

## Setting up the Software

### Open Raspberry pi I2C interface
Open a terminal window, then input:
```
sudo raspi-config
```
<img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Raspberry_pi/raspi-config.jpg" height="300">
Open Interface Option, then choose I2C option and open it.

### Calibration
The IMU in this project is a 9 DoF MARG. Accelerometer, gyroscope and magnetometer are implemented on it to sense acceleration, rotation speed and magnetic field along the three axes of the coordinate on itself. To successfully estimate the euler angle, calibration of all 9 data is required.

1. Calibrate accelerometer and gyroscope

When MARG is still on a flat stable, ideally, acceleration along x/y/z axis should be 0.0/0.0/-g (g is the gravity of that area) and rotation speed should be 0.0/0.0/0.0. However, in practice, these data not only has bias but also keep fluctuating. By using calibration, bias can be reduced. 

The basic idea of calibrating accelerometer and gyroscope is putting MARG on a flat table, then open and run file 9DoF_MARG_Madgwick_Filter/Raspberry_pi/Calibration/Gyro_Acc_Calibration.py. After that, keep receiving acceleration and rotation speed data for around 20 seconds. Then, taking average of each data, we can get the calibration number.

```
GYRO_X_OFFSET =  0.01363404
GYRO_Y_OFFSET =  0.00173217
GYRO_Z_OFFSET = -0.00597996
   
ACCEL_X_OFFSET = 0.01661877
ACCEL_Y_OFFSET = -0.35245354
ACCEL_Z_OFFSET = 10.07400186
```
Open file 9DoF_MARG_Madgwick_Filter/Raspberry_pi/Madwick_Filter/Madgwick_Filter.py. Find this part and replace the number.

2. Calibrate magnetometer

The calibration of magnetometer is more complex. It also has some restriction. If IMU works in a certain workspace, then magnetometer calibration is required to be performed in that place. The reason is that, ideally, we only want to receive geomagnetic field. But geomagnetic field is easily influenced by Ferromagnetic material, like iron or nickel. This is so called soft-iron distortion. Also, some cable with current or magnet produce magnetic field, which also influence the geomagnetic field we want. This is hard-iron distortion. The reason of calibration is to remove the soft-iron and hard-iron distortion. However, if the working place is changed, the distortion changes as well. Then the calibration cannot compensate the distortion.

To do that, run Raspberry_pi/Calibration/Mag_Calibration.py. This python program will run for 120 seconds. It processes raw magnetic field data. During this process, MPU9250 should be keep rotated randomly so that it can face to each direction at once. After 100 seconds (this duration can be edited in program), this program produces two images to show the result and two lists as shown below. Find this part in Raspberry_pi/Madwick_Filter/Madwick_filter.py and replace it.

```
magn_ellipsoid_center = np.array([-7.55084646, 24.20735906, -13.65776837])
magn_ellipsoid_transform = np.array([[ 0.97206544, -0.04793918, 0.00480131],
 [-0.04793918,  0.91558391, -0.00202764],
 [0.00480131, -0.00202764, 0.95006348]])
```

<img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Least_squares_ellipsoid_specific_fitting.jpg" height="350"> <img src="https://github.com/DonovanZhu/MPU9250_Madgwick_Filter/blob/master/Mag_Calibration.jpg" height="350">

In the above figures, red pints are raw data and bule points are calibrated data.

### Running Madgwick Filter
After calibration, it is time to run the filter. Running Madgwick_Filter.py.

## Notification

### MARG Position
Keep MARG away from large current unit as far as possible, including cables and motors. The current in these units is changing so that the magnetic field produced by them is also unstable. These unstable magnetic field cannot be calibrated.

### Madgwick Algorithm Parameter Tuning
In MadgwickAHRS.py file, there is a parameter "beta" in function MadgwickQuaternionUpdate(). This parameter is explained in the original paper in detail. Generally, it is the step of regression. If beta increase, attitude estimation performs faster but more oscillation comes out. When beta is small, regression slows down but the data is more stable.

## Reference
[1] [Adafruit LSM6DS Library](https://github.com/adafruit/Adafruit_LSM6DS) 

[2] [Adafruit LIS3MDL Library](https://github.com/adafruit/Adafruit_LIS3MDL)

[3] [9 Degree of Measurement Attitude and Heading Reference System for Sparkfun 9DOF Razor IMU and SparkFun 9DOF Sensor Stick](https://github.com/Razor-AHRS/razor-9dof-ahrs)

[4] [Madgwick Filter](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

[5] [Extended Kalman Filter Attitude Determination](https://www.thepoorengineer.com/en/attitude-determination/)

[6] [IMU Data Fusing: Complementary, Kalman, and Mahony Filter](http://www.olliw.eu/2013/imu-data-fusing/#refSM2)
