import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt

# Plot error of [dx, dy, dphi]
data = pd.read_csv("error_LV.CSV",low_memory=False)
data_all = np.array(data)

fig = plt.figure()
ax1 = fig.add_subplot(311)
ax1.plot(data_all[:,0], data_all[:,9],linewidth=1,color='green',label='Schätzfehler von $\Delta\it{x}_{Lidar}$')
ax1.plot(data_all[:,0], data_all[:,5],linewidth=1,color='red',label='Schätzfehler von $\Delta\it{x}_{Fusion LV}$')
ax1.set_ylabel('$\Delta\it{x}$ [m]',fontsize=14)
# plt.ylim((-0.4, 0.4))
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()

ax2 = fig.add_subplot(312)
ax2.plot(data_all[:,0], data_all[:,10],linewidth=1,color='green',label='Schätzfehler von $\Delta\it{y}_{Lidar}$')
ax2.plot(data_all[:,0], data_all[:,6],linewidth=1,color='red',label='Schätzfehler von $\Delta\it{y}_{Fusion LV}$')
ax2.set_ylabel('$\Delta\it{y}$ [m]',fontsize=14)
# plt.ylim((-0.4, 0.6))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()

ax3 = fig.add_subplot(313)
ax3.plot(data_all[:,0], data_all[:,8],linewidth=1,color='green',label='Schätzfehler von $\Delta\\varphi_{Lidar}$')
ax3.plot(data_all[:,0], data_all[:,4],linewidth=1,color='red',label='Schätzfehler von $\Delta\\varphi_{Fusion LV}$')
ax3.set_ylabel('$\Delta\\varphi$ [rad]',fontsize=14)
ax3.set_xlabel('Zeit [s]',fontsize=14)
# plt.ylim((-0.1, 0.1))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()
plt.show()






