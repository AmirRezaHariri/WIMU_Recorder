import numpy as np
import math
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q

np.seterr(divide='ignore', invalid='ignore')
class AHRS:

    def __init__(self, f, mr):
        self.Q = []
        self.freq = f
        self.magRef = mr
        self.Qi = [0, 0, 0, 0]

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def run(self, acc_data, gyr_data, mag_data):
        acc_data = 9.8 * np.array([acc_data], dtype=float)
        gyr_data = np.pi / 180 * np.array([gyr_data], dtype=float)
        mag_data = 1000 * np.array([mag_data], dtype=float)

        ekf = EKF(gyr=gyr_data, acc=acc_data, mag=mag_data,
                  ferquency=self.freq, magnetic_ref=self.magRef, frame='NED')

        self.Qi = acc2q(self.Qi)
        self.Qi = ekf.update(self.Qi, np.array(gyr_data[0], dtype=float),
                             np.array(acc_data[0], dtype=float), np.array(mag_data[0], dtype=float))
        a, b, c = self.euler_from_quaternion(self.Qi[0], self.Qi[1], self.Qi[2], self.Qi[3])
        # print(a, b, c)
        return [a, b, c, [self.Qi[0], self.Qi[1], self.Qi[2], self.Qi[3]]]
