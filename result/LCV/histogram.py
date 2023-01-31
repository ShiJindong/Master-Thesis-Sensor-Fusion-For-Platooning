import pandas as pd  
import numpy as np  
import matplotlib.pyplot as plt  
import math
import seaborn as sns


def normfun(x, mu, sigma):
    pdf = np.exp(-((x - mu) ** 2) / (2 * sigma ** 2)) / (sigma * np.sqrt(2 * np.pi))
    return pdf

def detect_outliers(data, outlier_threshold=0.2):
    num_outliers = 0
    n = 0
    
    for y in data:
        n += 1
        if np.abs(y) > outlier_threshold:
            num_outliers += 1
    
    ratio_outliers = num_outliers / n
    return ratio_outliers


def cal_mean(data, thres_left, thres_right):
    mean_d = np.mean(data)
    std_d = np.std(data)
    
    # calclulate mean
    mean = 0
    n = 0

    for y in data:
        if y < thres_right and y > thres_left:
            mean += y
            n += 1
    
    if n != 0:
        mean /= n
    else:
        mean = 0
    
    return mean


def cal_std(data, mean, thres_left, thres_right):
    mean_d = np.mean(data)
    std_d = np.std(data)

    # calclulate variance
    variance = 0
    n = 0

    for y in data:
        if y < thres_right and y > thres_left:
            variance += (y - mean) * (y - mean)
            n += 1

    if n > 1:
        variance /= (n - 1)
    else:
        variance = 0

    std = math.sqrt(variance)
    return std


def draw_histogram(data, data_name, color_name):
    ratio_outliers = detect_outliers(data, outlier_threshold=0.1)
    print('ratio_outliers of ', data_name, ' = ', ratio_outliers)

    # x = np.arange(-0.2, 0.2, 0.001)
    # y = normfun(x, mean, std)
    # plt.plot(x, y, color=color_name)

    # n, bins, patches = plt.hist(data,bins=1000,color=color_name,histtype='stepfilled',rwidth=0.1, density=1, alpha=1, label=data_name)
    # plt.plot(bins[:-1],n,'--')

    sns.set_palette("hls") 
    sns.histplot(data,color=color_name,kde=False, binwidth=0.005, stat="density", label=data_name)
   
    # plt.xlabel('Schätzfehler von $\Delta\it{x}$')



if __name__ == '__main__':
 
    data = pd.read_csv('error_LCV.CSV')

    fig = plt.figure()
    ax1 = fig.add_subplot(211)

    # data_lidar_dx = data['lidar_rel_pose_dx_estimate_error']
    # data_camera_dx = data['camera_rel_pose_dx_estimate_error']
    # data_fusion_dx = data['fusion_rel_pose_dx_estimate_error']

    # draw_histogram(data=data_lidar_dx, data_name='Schätzfehler von $\Delta\it{x}_{Lidar}$', color_name='green')
    # draw_histogram(data=data_camera_dx, data_name='Schätzfehler von $\Delta\it{x}_{Kamera}$', color_name='blue')
    # draw_histogram(data=data_fusion_dx, data_name='Schätzfehler von $\Delta\it{x}_{FusionLKV}$', color_name='red')


    data_lidar_dy = data['lidar_rel_pose_dy_estimate_error']
    data_camera_dy = data['camera_rel_pose_dy_estimate_error']
    data_fusion_dy = data['fusion_rel_pose_dy_estimate_error']

    draw_histogram(data=data_lidar_dy, data_name='Schätzfehler von $\Delta\it{y}_{Lidar}$', color_name='green')
    draw_histogram(data=data_camera_dy, data_name='Schätzfehler von $\Delta\it{y}_{Kamera}$', color_name='blue')
    draw_histogram(data=data_fusion_dy, data_name='Schätzfehler von $\Delta\it{y}_{FusionLKV}$', color_name='red')


    # data_lidar_dphi = data['lidar_rel_pose_dphi_estimate_error']
    # data_camera_dphi = data['camera_rel_pose_dphi_estimate_error']
    # data_fusion_dphi = data['fusion_rel_pose_dphi_estimate_error']

    # draw_histogram(data=data_lidar_dphi, data_name='Schätzfehler von $\Delta\\varphi_{Lidar}$', color_name='green')
    # draw_histogram(data=data_camera_dphi, data_name='Schätzfehler von $\Delta\\varphi_{Kamera}$', color_name='blue')
    # draw_histogram(data=data_fusion_dphi, data_name='Schätzfehler von $\Delta\\varphi_{FusionLKV}$', color_name='red')

    plt.xlim((-1, 1))
    # plt.ylim((-0.1, 0.6))
    plt.xlabel('',fontsize=14)
    plt.ylabel('Häufigkeit',fontsize=14)
    plt.legend(loc='upper right', prop = {'size':12})



    ax2 = fig.add_subplot(212)
    draw_histogram(data=data_lidar_dy, data_name='Schätzfehler von $\Delta\it{y}_{Lidar}$', color_name='green')
    draw_histogram(data=data_camera_dy, data_name='Schätzfehler von $\Delta\it{y}_{Kamera}$', color_name='blue')
    draw_histogram(data=data_fusion_dy, data_name='Schätzfehler von $\Delta\it{y}_{FusionLKV}$', color_name='red')

    plt.xlim((-1, 1))
    plt.ylim((-0, 0.5))
    plt.xlabel('Schätzfehler von $\Delta\it{y}$ [m]',fontsize=14)
    # plt.xlabel('Schätzfehler von $\Delta\\varphi$ [rad]',fontsize=14)
    plt.ylabel('Häufigkeit',fontsize=14)
    plt.legend(loc='upper right', prop = {'size':12})
    plt.show()

