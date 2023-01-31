import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

data = pd.read_csv("velocity_rot_rate_deviation.CSV",low_memory=False)
data_all = np.array(data)

fig = plt.figure()
ax1 = fig.add_subplot(311)

ax1.plot(data_all[:,0], data_all[:,14],linewidth=2,color='black',label='Wahre longitudinale Geschwindigkeit $\it{v}_{x,wahr}$')
ax1.plot(data_all[:,0], data_all[:,10],linewidth=2,color='green',label='Radgeschwindigkeit $\it{v}_{x,vl}$')
ax1.plot(data_all[:,0], data_all[:,11],linewidth=2,color='cyan',label='Radgeschwindigkeit $\it{v}_{x,vr}$')
ax1.plot(data_all[:,0], data_all[:,12],linewidth=2,color='blue',label='Radgeschwindigkeit $\it{v}_{x,hl}$')
ax1.plot(data_all[:,0], data_all[:,13],linewidth=2,color='red',label='Radgeschwindigkeit $\it{v}_{x,hr}$')

ax1.set_ylabel('$\it{v}_{x}$ [m/s]',fontsize=12)
ax1.set_xlabel('Zeit [s]',fontsize=12)
# plt.xlim((0, 20))
# plt.xlim((40, 50))
# plt.ylim((-1, 15))
plt.legend(loc='upper left', prop = {'size':10})
# plt.title('Darstellung des Radschlupfs des Rades')
plt.grid()


ax2 = fig.add_subplot(312)
ax2.plot(data_all[:,0], data_all[:,6],linewidth=2,color='green',label='Änderung der Rotationsgeschwindigkeit $\Delta\\omega_{vl}$')
ax2.plot(data_all[:,0], data_all[:,7],linewidth=2,color='cyan',label='Änderung der Rotationsgeschwindigkeit $\Delta\\omega_{vr}$')
ax2.plot(data_all[:,0], data_all[:,8],linewidth=2,color='blue',label='Änderung der Rotationsgeschwindigkeit $\Delta\\omega_{hl}$')
ax2.plot(data_all[:,0], data_all[:,9],linewidth=2,color='red',label='Änderung der Rotationsgeschwindigkeit $\Delta\\omega_{hr}$')

ax2.set_ylabel('$\Delta\\omega$ [rad/s]',fontsize=12)
ax2.set_xlabel('Zeit [s]',fontsize=12)
# plt.xlim((40, 50))
# plt.xlim((0, 20))
# plt.ylim((-1, 15))
plt.legend(loc='upper left', prop = {'size':10})
# plt.title('Kriterium I: Änderung der Rotationsgeschwindigkeit des Rades')
plt.grid()


ax3 = fig.add_subplot(313)
ax3.plot(data_all[:,0], data_all[:,2],linewidth=2,color='green',label='Abweichung der Radgeschwindigkeit $\Delta\it{v}_{vl}$')
ax3.plot(data_all[:,0], data_all[:,3],linewidth=2,color='cyan',label='Abweichung der Radgeschwindigkeit $\Delta\it{v}_{vr}$')
ax3.plot(data_all[:,0], data_all[:,4],linewidth=2,color='blue',label='Abweichung der Radgeschwindigkeit $\Delta\it{v}_{hl}$')
ax3.plot(data_all[:,0], data_all[:,5],linewidth=2,color='red',label='Abweichung der Radgeschwindigkeit $\Delta\it{v}_{hr}$')

# plt.xlim((0, 20))
# plt.xlim((40, 50))
# plt.ylim((-5, 2))

ax3.set_ylabel('$\Delta\it{v}$ [m/s]',fontsize=12)
ax3.set_xlabel('Zeit [s]',fontsize=12)
plt.legend(loc='lower left', prop = {'size':10})
# plt.title('Kriterium II: Abweichung der Radgeschwindigkeit von der prädizierten Radgeschwindigkeit')
plt.grid()

plt.show()






