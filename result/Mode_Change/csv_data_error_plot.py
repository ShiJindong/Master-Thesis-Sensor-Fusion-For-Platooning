import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt

# Plot error of [dx, dy, dphi]
data = pd.read_csv("error_mode_change.CSV",low_memory=False)
data_all = np.array(data)

fig = plt.figure()
ax1 = fig.add_subplot(311)
ax1.plot(data_all[1:4000,0], data_all[1:4000,2],linewidth=1,color='blue',label='Schätzfehler von $\Delta\it{x}_{Kamera}$')
ax1.plot(data_all[8000:16000,0], data_all[8000:16000,2],linewidth=1,color='blue')
ax1.plot(data_all[18200:20400,0], data_all[18200:20400,2],linewidth=1,color='blue')

ax1.plot(data_all[1:4000,0], data_all[1:4000,9],linewidth=1,color='green',label='Schätzfehler von $\Delta\it{x}_{Lidar}$')
ax1.plot(data_all[6000:14000,0], data_all[6000:14000,9],linewidth=1,color='green')
ax1.plot(data_all[16000:18200,0], data_all[16000:18200,9],linewidth=1,color='green')

ax1.plot(data_all[:,0], data_all[:,5],linewidth=1,color='red',label='Schätzfehler von $\Delta\it{x}_{Fusion}$')

ax1.set_ylabel('$\Delta\it{x}$ [m]',fontsize=14)
plt.ylim((-0.6, 0.5))
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()
# plt.title('Schätzfehler der relativen Pose von Kamera-Tracker, Lidar-Tracker und Fusionsalgorithmus', fontsize=14)

ax2 = fig.add_subplot(312)

ax2.plot(data_all[1:4000,0], data_all[1:4000,3],linewidth=1,color='blue',label='Schätzfehler von $\Delta\it{y}_{Kamera}$')
ax2.plot(data_all[8000:16000,0], data_all[8000:16000,3],linewidth=1,color='blue')
ax2.plot(data_all[18200:20400,0], data_all[18200:20400,3],linewidth=1,color='blue')

ax2.plot(data_all[1:4000,0], data_all[1:4000,10],linewidth=1,color='green',label='Schätzfehler von $\Delta\it{y}_{Lidar}$')
ax2.plot(data_all[6000:14000,0], data_all[6000:14000,10],linewidth=1,color='green')
ax2.plot(data_all[16000:18200,0], data_all[16000:18200,10],linewidth=1,color='green')

ax2.plot(data_all[:,0], data_all[:,6],linewidth=1,color='red',label='Schätzfehler von $\Delta\it{y}_{Fusion}$')

ax2.set_ylabel('$\Delta\it{y}$ [m]',fontsize=14)
plt.ylim((-0.3, 0.5))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()

ax3 = fig.add_subplot(313)

ax3.plot(data_all[1:4000,0], data_all[1:4000,1],linewidth=1,color='blue',label='Schätzfehler von $\Delta\\varphi_{Kamera}$')
ax3.plot(data_all[8000:16000,0], data_all[8000:16000,1],linewidth=1,color='blue')
ax3.plot(data_all[18200:20400,0], data_all[18200:20400,1],linewidth=1,color='blue')

ax3.plot(data_all[1:4000,0], data_all[1:4000,8],linewidth=1,color='green',label='Schätzfehler von $\Delta\\varphi_{Lidar}$')
ax3.plot(data_all[6000:14000,0], data_all[6000:14000,8],linewidth=1,color='green')
ax3.plot(data_all[16000:18200,0], data_all[16000:18200,8],linewidth=1,color='green')

ax3.plot(data_all[:,0], data_all[:,4],linewidth=1,color='red',label='Schätzfehler von $\Delta\\varphi_{Fusion}$')
ax3.set_ylabel('$\Delta\\varphi$ [rad]',fontsize=14)
ax3.set_xlabel('Zeit [s]',fontsize=14)
plt.ylim((-0.2, 0.3))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()
plt.show()






