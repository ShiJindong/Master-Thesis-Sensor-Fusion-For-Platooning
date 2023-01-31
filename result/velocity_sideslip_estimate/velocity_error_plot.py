import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


# data = pd.read_csv("velocity_sideslip_curve.CSV",low_memory=False)
# data_all = np.array(data)

fig = plt.figure()
# ax1 = fig.add_subplot(411)
# ax1.plot(data_all[:,0], data_all[:,5],linewidth=2,color='black',label='Tatsächliche Werte der longitudinalen Geschwindigkeit des verfolgenden Busses $\it{v}_{x1,wahr}$')
# ax1.plot(data_all[:,0], data_all[:,1],linewidth=2,color='red',label='Schätzwerte der longitudinalen Geschwindigkeit des verfolgenden Busses $\it{v}_{x1,est}$')
# ax1.set_ylabel('$\it{v}_{x1}$ [m/s]',fontsize=14)
# plt.legend(loc='lower right', prop = {'size':12})
# plt.grid()

# ax2 = fig.add_subplot(412)
# ax2.plot(data_all[:,0], data_all[:,7],linewidth=2,color='black',label='Tatsächliche Werte der longitudinalen Geschwindigkeit des führenden Busses $\it{v}_{x2,wahr}$')
# ax2.plot(data_all[:,0], data_all[:,3],linewidth=2,color='red',label='Schätzwerte der longitudinalen Geschwindigkeit des führenden Busses $\it{v}_{x2,est}$')
# ax2.set_ylabel('$\it{v}_{x2}$ [m/s]',fontsize=14)
# plt.legend(loc='lower right', prop = {'size':12})
# plt.grid()

data_error = pd.read_csv("velocity_sideslip_error.CSV",low_memory=False)
data_all_error = np.array(data_error)

ax1 = fig.add_subplot(211)
ax1.plot(data_all_error[:,0], data_all_error[:,1],linewidth=2,color='orange',label='Schätzfehler der longitudinalen Geschwindigkeit des verfolgenden Busses $\it{v}_{x1,est}$')
ax1.set_ylabel('Schätzfehler von $\it{v}_{x1,est}$ [m/s]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()

ax2 = fig.add_subplot(212)
ax2.plot(data_all_error[:,0], data_all_error[:,5],linewidth=2,color='cyan',label='Schätzfehler der longitudinalen Geschwindigkeit des führenden Busses $\it{v}_{x2,est}$')
ax2.set_ylabel('Schätzfehler von $\it{v}_{x2,est}$ [m/s]',fontsize=14)
ax2.set_xlabel('Zeit [s]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()
plt.show()






