import numpy as np
import matplotlib.pyplot as plt
import os

plt.style.use('ggplot')

home = "/home/jorhabib/Documents/octave/MIQP/"
X_kn = np.loadtxt(home + "solution.txt")
time = X_kn[:,0]
a_x = X_kn[:,1]
a_y = X_kn[:,2]
b_x = X_kn[:,3]
b_y = X_kn[:,4]
alpha_x = X_kn[:,5]
alpha_y = X_kn[:,6]
beta_x = X_kn[:,7]
beta_y = X_kn[:,8]
delta = X_kn[:,9]
gamma = X_kn[:,10]
u_x = X_kn[:,11]
u_y = X_kn[:,12]

# CoM velocity
# with plt.style.context('fivethirtyeight'):
ax1 = plt.subplot(6,1,1)
plt.title('Upper and lower bounds in the x axis')
ax1.plot(time, a_x, 'o--', label='a_x', linewidth = 3)
ax1.plot(time, b_x, 'o-', label='b_x', linewidth = 3)
ax1.set_ylabel('[m]')
ax1.autoscale(enable=True, axis='x', tight=True)
ax1.legend()

ax2 = plt.subplot(6,1,2)
plt.title('Rising/falling edges in x axis')
ax2.plot(time, alpha_x, 'o--',  label='alpha_x', linewidth = 3)
ax2.plot(time, beta_x, label='beta_x', linewidth = 3)
ax2.set_ylabel('binary')
ax2.autoscale(enable=True, axis='x', tight=True)
ax2.legend()

ax3 = plt.subplot(6,1,3)
plt.title('Upper and lower bounds in the y axis')
ax3.plot(time, a_y, '--',  label='a_y', linewidth = 3)
ax3.plot(time, b_y, label='b_y', linewidth = 3)
ax3.set_ylabel('[m]')
ax3.autoscale(enable=True, axis='x', tight=True)
ax3.legend()

ax4 = plt.subplot(6,1,4)
plt.title('Rising/falling edges in y axis')
ax4.plot(time, alpha_y, '--', label='alpha_y', linewidth = 3)
ax4.plot(time, beta_y, label='beta_y', linewidth = 3)
ax4.set_ylabel('Binary')
ax4.autoscale(enable=True, axis='x', tight=True)
ax4.legend()

ax5 = plt.subplot(6,1,5)
plt.title('Support Phase')
ax5.plot(time, gamma, label='gamma', linewidth = 3)
ax5.set_ylabel('DS:1, SS:0')
ax5.autoscale(enable=True, axis='x', tight=True)
ax5.legend()

ax6 = plt.subplot(6,1,6)
plt.title('Potential Change from DS to SS')
ax6.plot(time, delta, label='delta', linewidth = 3)
ax6.set_ylabel('Binary')
ax6.autoscale(enable=True, axis='x', tight=True)
ax6.legend()

# plotting
# plt.tight_layout()
plt.show()

answer = input("Want to delete the data? [y/n]")
if answer == 'y':
    os.remove(home + "solution.txt");

def functionname( plotTitle, values ):
   return;
