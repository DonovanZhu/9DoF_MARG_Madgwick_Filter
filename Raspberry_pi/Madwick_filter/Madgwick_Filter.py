from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from scipy.spatial.transform import Rotation as R
import board
import numpy as np
import time
import math

accel_gyro = LSM6DS(board.I2C())
magnetic = LIS3MDL(board.I2C())

GRAVITY = 9.802 # The gravity acceleration in New York City

# Calibration outcomes

GYRO_X_OFFSET =  0.01363404
GYRO_Y_OFFSET =  0.00173217
GYRO_Z_OFFSET = -0.00597996
   
ACCEL_X_OFFSET = 0.01661877
ACCEL_Y_OFFSET = -0.35245354
ACCEL_Z_OFFSET = 10.07400186
    
   


magn_ellipsoid_center = np.array([-7.55084646, 24.20735906, -13.65776837])
magn_ellipsoid_transform = np.array([[ 0.97206544, -0.04793918, 0.00480131],
 [-0.04793918,  0.91558391, -0.00202764],
 [0.00480131, -0.00202764, 0.95006348]])

time_former = 0

q = [1, 0, 0, 0]

def read_sensors():
    acc = np.array(accel_gyro.acceleration)
    mag = np.array(magnetic.magnetic)
    gyr = np.array(accel_gyro.gyro)
    return acc, gyr, mag

def compensate_sensor_errors(acc, gyr, mag):
    # Compensate accelerometer error
    acc[0] -= ACCEL_X_OFFSET
    acc[1] -= ACCEL_Y_OFFSET
    acc[2] -= ACCEL_Z_OFFSET - GRAVITY

    # Compensate magnetometer error
    mag_tmp = mag - magn_ellipsoid_center
    mag = magn_ellipsoid_transform.dot(mag_tmp)

    # Compensate gyroscope error
    gyr[0] -= GYRO_X_OFFSET
    gyr[1] -= GYRO_Y_OFFSET
    gyr[2] -= GYRO_Z_OFFSET
    
    return acc, gyr, mag


def MadgwickQuaternionUpdate(acc, gyr, mag, deltat):

    beta = 3
    q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
    ax, ay, az = acc[0], acc[1], acc[2]
    gx, gy, gz = gyr[0], gyr[1], gyr[2]
    mx, my, mz = mag[0], mag[1], mag[2]
    # Auxiliary variables to avoid repeated arithmetic
    _2q1 = 2 * q1
    _2q2 = 2 * q2
    _2q3 = 2 * q3
    _2q4 = 2 * q4
    _2q1q3 = 2 * q1 * q3
    _2q3q4 = 2 * q3 * q4
    q1q1 = q1 * q1
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q1q4 = q1 * q4
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q2q4 = q2 * q4
    q3q3 = q3 * q3
    q3q4 = q3 * q4
    q4q4 = q4 * q4

    # Normalise accelerometer measurement
    norm = 1 / math.sqrt(ax * ax + ay * ay + az * az)
    ax *= norm
    ay *= norm
    az *= norm

    # Normalise magnetometer measurement
    norm = 1 / math.sqrt(mx * mx + my * my + mz * mz)
    mx *= norm
    my *= norm
    mz *= norm

    # Reference direction of Earth's magnetic field
    _2q1mx = 2 * q1 * mx
    _2q1my = 2 * q1 * my
    _2q1mz = 2 * q1 * mz
    _2q2mx = 2 * q2 * mx
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
    _2bx = math.sqrt(hx * hx + hy * hy)
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
    _4bx = 2 * _2bx
    _4bz = 2 * _2bz

    # Gradient decent algorithm corrective step
    s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
    norm = 1 / norm
    s1 *= norm
    s2 *= norm
    s3 *= norm
    s4 *= norm

    # Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4

    # Integrate to yield quaternion
    q1 += qDot1 * deltat
    q2 += qDot2 * deltat
    q3 += qDot3 * deltat
    q4 += qDot4 * deltat
    norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
    norm = 1 / norm;
    q[0] = q1 * norm
    q[1] = q2 * norm
    q[2] = q3 * norm
    q[3] = q4 * norm


if __name__ == '__main__':
    time_former = time.clock()
    while True:
        acc ,gyr, mag = read_sensors()

        acc ,gyr, mag = compensate_sensor_errors(acc ,gyr, mag)

        time_now = time.clock()
        deltat = time_now - time_former;
        time_former = time_now;
        if (0.01 - deltat > 0):
            time.sleep(0.01 - deltat)
        MadgwickQuaternionUpdate(acc, gyr, mag, 0.01)
        
        r = R.from_quat(q)
        eular = r.as_euler('zyx', degrees=True)
        
        eular[2] -= 60; # 0.8 this used for compensating the angle between magenatic north and geographic north
        if (eular[2] < -180):
            eular[2] += 360

        print(eular, deltat)

