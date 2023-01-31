import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt

# Plot error of [dx, dy, dphi]
data = pd.read_csv("error_LCV.CSV",low_memory=False)
data_all = np.array(data)

fig = plt.figure()
ax1 = fig.add_subplot(411)
ax1.plot(data_all[:,0], data_all[:,2],linewidth=1,color='blue',label='Schätzfehler von $\Delta\it{x}_{Kamera}$')
ax1.set_ylabel('$\Delta\it{x}$ [m]',fontsize=14)
# plt.ylim((-0.5, 0.5))
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()
# plt.title('Schätzfehler der relativen Pose von Kamera-Tracker, Lidar-Tracker und Fusionsalgorithmus', fontsize=14)

ax2 = fig.add_subplot(412)
ax2.plot(data_all[:,0], data_all[:,3],linewidth=1,color='blue',label='Schätzfehler von $\Delta\it{y}_{Kamera}$')
ax2.set_ylabel('$\Delta\it{y}$ [m]',fontsize=14)
# plt.ylim((-0.4, 0.8))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()

ax3 = fig.add_subplot(413)
ax3.plot(data_all[:,0], data_all[:,1],linewidth=1,color='blue',label='Schätzfehler von $\Delta\\varphi_{Kamera}$')
ax3.set_ylabel('$\Delta\\varphi$ [rad]',fontsize=14)
ax3.set_xlabel('Zeit [s]',fontsize=14)
ax3.set_xlabel('',fontsize=14)
# plt.ylim((-0.2, 0.2))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()

ax4 = fig.add_subplot(414)
ax4.plot(data_all[:,0], data_all[:,11],linewidth=1,color='orange',label='Anzahl der detektierten Aruco-Markers vom Kamera-Tracker')
ax4.set_ylabel('Anzahl der Aruco-Markers',fontsize=14)
ax4.set_xlabel('Zeit [s]',fontsize=14)
plt.ylim((-1, 5))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()


plt.show()






