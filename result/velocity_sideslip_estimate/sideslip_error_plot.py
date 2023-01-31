import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


data = pd.read_csv("velocity_sideslip_curve.CSV",low_memory=False)
data_all = np.array(data)

fig1 = plt.figure()
ax0 = fig1.add_subplot(111)
ax0.plot(data_all[:,0], data_all[:,2],linewidth=2,color='green',label='Schätzwert des Schwimmwinkels des verfolgenden Busses $\\beta_{1,est}$')
ax0.plot(data_all[:,0], data_all[:,6],linewidth=2,color='red',label='wahrer Wert des Schwimmwinkels des verfolgenden Busses $\\beta_{1,wahr}$')
ax0.plot(data_all[:,0], data_all[:,4],linewidth=2,color='blue',label='Schätzwert des Schwimmwinkels des führenden Busses $\\beta_{2,est}$')
ax0.plot(data_all[:,0], data_all[:,8],linewidth=2,color='gold',label='wahrer Wert des Schwimmwinkels des führenden Busses $\\beta_{2,wahr}$')
plt.ylim((-0.4, 0.3))

ax0.set_ylabel('Schwimmwinkel [rad]',fontsize=14)
ax0.set_xlabel('Zeit [s]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()
plt.show()



data_error = pd.read_csv("velocity_sideslip_error.CSV",low_memory=False)
data_all_error = np.array(data_error)

fig2 = plt.figure()

ax1 = fig2.add_subplot(211)
ax1.plot(data_all_error[:,0], data_all_error[:,2],linewidth=2,color='orange',label='Schätzfehler des Schwimmwinkels des verfolgenden Busses $\\beta_{1,est}$')
ax1.set_ylabel('Schätzfehler von $\\beta_{1,est}$ [rad]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()

ax2 = fig2.add_subplot(212)
ax2.plot(data_all_error[:,0], data_all_error[:,6],linewidth=2,color='cyan',label='Schätzfehler des Schwimmwinkels des führenden Busses $\\beta_{2,est}$')
ax2.set_ylabel('Schätzfehler von $\\beta_{2,est}$ [rad]',fontsize=14)
ax2.set_xlabel('Zeit [s]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()
plt.show()








