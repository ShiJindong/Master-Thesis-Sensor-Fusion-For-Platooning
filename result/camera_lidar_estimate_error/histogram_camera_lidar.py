import pandas as pd  
import numpy as np  
import matplotlib.pyplot as plt  
import math
import seaborn as sns


def draw_histogram(data, data_name, color_name):
    sns.set_palette("hls") 
    sns.histplot(data,color=color_name,kde=False, binwidth=0.005, stat="density", label=data_name)




if __name__ == '__main__':
 
    data = pd.read_csv('error_LCV.CSV')

    fig = plt.figure()
    ax1 = fig.add_subplot(311)

    data_lidar_dx = data['lidar_rel_pose_dx_estimate_error']
    data_camera_dx = data['camera_rel_pose_dx_estimate_error']

    draw_histogram(data=data_lidar_dx, data_name='Schätzfehler von $\Delta\it{x}_{Lidar}$', color_name='green')
    draw_histogram(data=data_camera_dx, data_name='Schätzfehler von $\Delta\it{x}_{Kamera}$', color_name='blue')

    plt.xlim((-2, 2))
    # plt.ylim((-0, 0.4))
    plt.xlabel('Schätzfehler von $\Delta\it{x}$ [m]',fontsize=12)
    plt.ylabel('Häufigkeit',fontsize=14)
    plt.legend(loc='upper right', prop = {'size':12})


    ax2 = fig.add_subplot(312)

    data_lidar_dy = data['lidar_rel_pose_dy_estimate_error']
    data_camera_dy = data['camera_rel_pose_dy_estimate_error']

    draw_histogram(data=data_lidar_dy, data_name='Schätzfehler von $\Delta\it{y}_{Lidar}$', color_name='green')
    draw_histogram(data=data_camera_dy, data_name='Schätzfehler von $\Delta\it{y}_{Kamera}$', color_name='blue')

    plt.xlim((-2, 2))
    # plt.ylim((-0, 0.4))
    plt.xlabel('Schätzfehler von $\Delta\it{y}$ [m]',fontsize=12)
    plt.ylabel('Häufigkeit',fontsize=14)
    plt.legend(loc='upper right', prop = {'size':12})


    ax3 = fig.add_subplot(313)

    data_lidar_dphi = data['lidar_rel_pose_dphi_estimate_error']
    data_camera_dphi = data['camera_rel_pose_dphi_estimate_error']

    draw_histogram(data=data_lidar_dphi, data_name='Schätzfehler von $\Delta\\varphi_{Lidar}$', color_name='green')
    draw_histogram(data=data_camera_dphi, data_name='Schätzfehler von $\Delta\\varphi_{Kamera}$', color_name='blue')

    plt.xlim((-2, 2))
    # plt.ylim((-0, 0.3))
    plt.xlabel('Schätzfehler von $\Delta\\varphi$ [rad]',fontsize=12)
    plt.ylabel('Häufigkeit',fontsize=14)
    plt.legend(loc='upper right', prop = {'size':12})
    plt.show()

