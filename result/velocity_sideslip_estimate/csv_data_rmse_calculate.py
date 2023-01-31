from tkinter import N
import pandas as pd
import numpy as np
import math


def cal_rmse(data):
    rmse = 0
    n = 0

    for error in data:
        rmse += error * error
        n += 1

    if n != 0:
        rmse /= n
        rmse = math.sqrt(rmse)
    else:
        rmse = 0

    return rmse


####################################################################################################
data_error = pd.read_csv("velocity_sideslip_error.CSV",low_memory=False)
data_all_error = np.array(data_error)

rmse_vx1 = cal_rmse(data_all_error[:, 1])
rmse_vx2 = cal_rmse(data_all_error[:, 5])
print('rmse_vx1: \t', rmse_vx1)
print('rmse_vx2: \t', rmse_vx2)


rmse_beta1 = cal_rmse(data_all_error[:, 2])
rmse_beta2 = cal_rmse(data_all_error[:, 6])
print('rmse_beta1: \t', rmse_beta1)
print('rmse_beta2: \t', rmse_beta2)