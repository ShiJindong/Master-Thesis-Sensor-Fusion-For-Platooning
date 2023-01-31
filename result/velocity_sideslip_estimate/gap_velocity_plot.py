import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

data = pd.read_csv("gap_velocity.CSV",low_memory=False)
data_all = np.array(data)

fig = plt.figure()
ax1 = fig.add_subplot(311)
ax1.plot(data_all[:,0], data_all[:,10],linewidth=2,color='violet',label='Tatsächliche Werte des longitudinalen Abstands zweier Busse $\Delta\it{x}_{Wahr}$')

ax1.set_ylabel('$\Delta\it{x}$ [m]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()


ax2 = fig.add_subplot(312)
ax2.plot(data_all[:,0], data_all[:,11],linewidth=2,color='brown',label='Tatsächliche Werte des lateralen Versatzes zweier Busse $\Delta\it{y}_{Wahr}$')
ax2.set_ylabel('$\Delta\it{y}$ [m]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()


ax3 = fig.add_subplot(313)
ax3.plot(data_all[:,0], data_all[:,7],linewidth=2,color='pink',label='Tatsächliche Werte der longitudinalen Geschwindigkeit des verfolgenden Busses $\it{v}_{x1,Wahr}$')
ax3.plot(data_all[:,0], data_all[:,8],linewidth=2,color='lime',label='Tatsächliche Werte der longitudinalen Geschwindigkeit des führenden Busses $\it{v}_{x2,Wahr}$')
# plt.ylim((-1, 1))
# plt.xlim((0.0, 55))
ax3.set_ylabel('Longitudinale Geschwindigkeit [m/s]',fontsize=12)
ax2.set_xlabel('Zeit [s]',fontsize=14)
plt.legend(loc='lower right', prop = {'size':12})
plt.grid()
plt.show()






