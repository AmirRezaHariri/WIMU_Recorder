import time
import matplotlib.pyplot as plt
import numpy as np


#####################################
# Mag Calibration Functions
#####################################
#
def outlier_removal(x_ii, y_ii):
    x_diff = np.append(0.0, np.diff(x_ii))  # looking for outliers
    stdev_amt = 5.0  # standard deviation multiplier
    x_outliers = np.where(np.abs(x_diff) > np.abs(np.mean(x_diff)) + \
                          (stdev_amt * np.std(x_diff)))
    x_inliers = np.where(np.abs(x_diff) < np.abs(np.mean(x_diff)) + \
                         (stdev_amt * np.std(x_diff)))
    y_diff = np.append(0.0, np.diff(y_ii))  # looking for outliers
    y_outliers = np.where(np.abs(y_diff) > np.abs(np.mean(y_diff)) + \
                          (stdev_amt * np.std(y_diff)))
    y_inliers = np.abs(y_diff) < np.abs(np.mean(y_diff)) + \
                (stdev_amt * np.std(y_diff))
    if len(x_outliers) != 0:
        x_ii[x_outliers] = np.nan  # null outlier
        y_ii[x_outliers] = np.nan  # null outlier
    if len(y_outliers) != 0:
        y_ii[y_outliers] = np.nan  # null outlier
        x_ii[y_outliers] = np.nan  # null outlier
    return x_ii, y_ii


def mag_cal(inp):
    mag_cal_rotation_vec = inp
    cal_rot_indices = [[0, 1], [1, 2], [0, 2]]
    for i in range(0, 3):
        mag_cal_rotation_vec[i] = mag_cal_rotation_vec[i][20:]  # throw away first few points (buffer clearing)
    mag_cal_rotation_vec = np.array(mag_cal_rotation_vec)  # make numpy array
    ak_fit_coeffs = []
    indices_to_save = [0, 0, 1]  # indices to save as offsets
    for mag_ii, mags in enumerate(mag_cal_rotation_vec):
        mags = np.array(mags)  # mag numpy array
        x, y = mags[:, cal_rot_indices[mag_ii][0]], \
               mags[:, cal_rot_indices[mag_ii][1]]  # sensors to analyze
        x, y = outlier_removal(x, y)  # outlier removal
        y_0 = (np.nanmax(y) + np.nanmin(y)) / 2.0  # y-offset
        x_0 = (np.nanmax(x) + np.nanmin(x)) / 2.0  # x-offset
        ak_fit_coeffs.append([x_0, y_0][indices_to_save[mag_ii]])  # append to offset

    # change this for soft iron
    identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    return ak_fit_coeffs, identity

# a = []
# b = []
# c = []
# for i in range(0, 100):
#     a.append(np.random.rand(3))
#     b.append(np.random.rand(3))
#     c.append(np.random.rand(3))
# d = [a, b, c]
# mag_cal(d)