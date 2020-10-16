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
    acceleration = accel_gyro.acceleration
    gyro = accel_gyro.gyro
    avg_gyro += np.array(gyro)
    avg_acc  += np.array(acceleration)
    #avg_gyro[0], avg_gyro[1], avg_gyro[2] += gyro[0], gyro[1], gyro[2]
    #avg_acc[0], avg_acc[1], avg_acc[2] += acceleration[0], acceleration[1], acceleration[2]
    print(avg_acc / loop_num)
    print(avg_gyro / loop_num)
    print("")
    loop_num += 1

    #time.sleep(0.5)

# [ 0.06653735 -0.11510433 10.08015258]
# [ 0.01383991  0.00135993 -0.00649945]

