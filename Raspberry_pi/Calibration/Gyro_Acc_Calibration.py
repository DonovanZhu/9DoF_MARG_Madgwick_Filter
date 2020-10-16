import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
import numpy as np

from adafruit_lis3mdl import LIS3MDL
 
accel_gyro = LSM6DS(board.I2C())
mag = LIS3MDL(board.I2C())

avg_gyro = np.array([0.0, 0.0, 0.0])
avg_acc  = np.array([0.0, 0.0, 0.0])

loop_num = 1
while True:
    # read data and calculate the average
    acceleration = accel_gyro.acceleration
    gyro = accel_gyro.gyro
    avg_gyro += np.array(gyro)
    avg_acc  += np.array(acceleration)

    print(avg_acc / loop_num)
    print(avg_gyro / loop_num)
    print("")
    loop_num += 1
