import time
import board

from adafruit_lis3mdl import LIS3MDL
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


mag = LIS3MDL(board.I2C())
magnetic = mag.magnetic
magnetic_data = np.array(magnetic)

# fit all data to an ellipsoid
def ellipsoid_fit(x, y, z):
    D = np.array([ np.multiply(x, x), np.multiply(y, y), np.multiply(z, z), 2 * np.multiply(x, y), 2 * np.multiply(x, z), 2 * np.multiply(y, z), 2 * x, 2 * y, 2 * z ])
    m_1 = np.linalg.inv(np.dot(D, D.transpose()))
    m_2 = np.dot(D, np.full((len(x), 1), 1))
    v = np.dot(m_1, m_2)
    v = v.transpose()[0]
    A = np.array([[v[0], v[3], v[4], v[6]],
                  [v[3], v[1], v[5], v[7]],
                  [v[4], v[5], v[2], v[8]],
                  [v[6], v[7], v[8], -1 ]])

    center = np.dot(-np.linalg.inv(A[0:3, 0:3]), np.array([[v[6]], [v[7]], [v[8]]]))

    T = np.identity(4)
    
    T[3, 0], T[3, 1], T[3, 2] = center[0], center[1], center[2]
    
    R = np.dot(T, np.dot(A, T.transpose()))
    
    evals, evecs  = np.linalg.eig( np.linalg.inv(R[0:3, 0:3]) * -R[3, 3])

    radii = np.sqrt(np.reciprocal(evals))
    
    
    return center, radii, evecs, v

if __name__ == "__main__":
    
    # read data for 100 seconds
    t0 = time.perf_counter()
    while True:
        magnetic = mag.magnetic
        magnetic_data = np.vstack([magnetic_data,magnetic])
        t1 = time.perf_counter() - t0
        
        if (t1 > 100):
            break
    # np.savetxt('magnetometer.csv', magnetic_data, delimiter=',', fmt='%1.7f')
    
    Mag_x = magnetic_data[:,0]
    Mag_y = magnetic_data[:,1]
    Mag_z = magnetic_data[:,2]
    e_center, e_radii, e_eigenvecs, e_algebraic = ellipsoid_fit(Mag_x, Mag_y, Mag_z)
    
    
    S = np.array([Mag_x - e_center[0], Mag_y - e_center[1], Mag_z - e_center[2]])

    scale = np.dot(np.linalg.inv(np.array([[e_radii[0], 0, 0], [0, e_radii[1], 0], [0, 0, e_radii[2]]])), np.identity(3)) * min(e_radii)

    e_map = e_eigenvecs.transpose()
    invmap = e_eigenvecs
    comp = np.dot(invmap, np.dot(scale, e_map))
    
    S = np.dot(comp, S)
    
    print('magn_ellipsoid_center = ', e_center)
    print('magn_ellipsoid_transform = ', comp)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(Mag_x, Mag_y, Mag_z, c='r')
    ax.scatter(S[0], S[1], S[2], c='b')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()
