import pandas as pd
import numpy as np
import csv
import matplotlib.pyplot as plt

# Plot error of [dx, dy, dphi]
data = pd.read_csv("error_CV.CSV",low_memory=False)
data_all = np.array(data)

fig = plt.figure()
ax1 = fig.add_subplot(311)
ax1.plot(data_all[:,0], data_all[:,2],linewidth=1,color='blue',label='Schätzfehler von $\Delta\it{x}_{Kamera}$')
ax1.plot(data_all[:,0], data_all[:,5],linewidth=1,color='red',label='Schätzfehler von $\Delta\it{x}_{Fusion KV}$')
ax1.set_ylabel('$\Delta\it{x}$ [m]',fontsize=14)
plt.ylim((-0.4, 0.4))
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()

ax2 = fig.add_subplot(312)
ax2.plot(data_all[:,0], data_all[:,3],linewidth=1,color='blue',label='Schätzfehler von $\Delta\it{y}_{Kamera}$')
ax2.plot(data_all[:,0], data_all[:,6],linewidth=1,color='red',label='Schätzfehler von $\Delta\it{y}_{Fusion KV}$')
ax2.set_ylabel('$\Delta\it{y}$ [m]',fontsize=14)
plt.ylim((-0.4, 0.6))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()

ax3 = fig.add_subplot(313)
ax3.plot(data_all[:,0], data_all[:,1],linewidth=1,color='blue',label='Schätzfehler von $\Delta\\varphi_{Kamera}$')
ax3.plot(data_all[:,0], data_all[:,4],linewidth=1,color='red',label='Schätzfehler von $\Delta\\varphi_{Fusion KV}$')
ax3.set_ylabel('$\Delta\\varphi$ [rad]',fontsize=14)
ax3.set_xlabel('Zeit [s]',fontsize=14)
plt.ylim((-0.2, 0.2))
plt.legend(loc='upper right', prop = {'size':12})
plt.grid()
plt.show()






