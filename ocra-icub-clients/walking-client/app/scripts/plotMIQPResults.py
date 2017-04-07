import numpy as np
import matplotlib.pyplot as plt
import os

plt.style.use('ggplot')

home = "/home/jorhabib/Documents/octave/MIQP/"
X_kn = np.loadtxt(home + "solutionInPreview.txt")
P_kn = np.loadtxt(home + "CoPinPreview.txt")
R_kn = np.loadtxt(home + "BoSinPreview.txt")
H_kn = np.loadtxt(home + "CoMinPreview.txt")
r_x = R_kn[:,0]
r_y = R_kn[:,1]
p_x = P_kn[:,0]
p_y = P_kn[:,1]
a_x = X_kn[:,0]
a_y = X_kn[:,1]
b_x = X_kn[:,2]
b_y = X_kn[:,3]
alpha_x = X_kn[:,4]
alpha_y = X_kn[:,5]
beta_x = X_kn[:,6]
beta_y = X_kn[:,7]
delta = X_kn[:,8]
gamma = X_kn[:,9]
u_x = X_kn[:,10]
u_y = X_kn[:,11]
h_x = H_kn[:,0]
h_y = H_kn[:,1]
dh_x = H_kn[:,2]
dh_y = H_kn[:,3]
ddh_x = H_kn[:,4]
ddh_y = H_kn[:,5]

# CoM velocity
# with plt.style.context('fivethirtyeight'):
ax1 = plt.subplot(5,2,1)
plt.title('Upper and lower bounds in the x axis')
ax1.plot( a_x, '--o', label='a_x', linewidth = 3)
ax1.plot( b_x, '-o', label='b_x', linewidth = 3)
ax1.plot( r_x, '--', label='r_x',linewidth = 3)
ax1.set_ylabel('[m]')
ax1.autoscale(enable=True, axis='x', tight=True)
ax1.legend()

ax2 = plt.subplot(5,2,2)
plt.title('Rising/falling edges in x axis')
ax2.plot( alpha_x, '--o', label='alpha_x', linewidth = 3)
ax2.plot( beta_x, '-o', label='beta_x', linewidth = 3)
ax2.set_ylim([-0.2,1.2])
ax2.set_ylabel('binary')
ax2.autoscale(enable=True, axis='x', tight=True)
ax2.legend()

ax3 = plt.subplot(5,2,3)
plt.title('Upper and lower bounds in the y axis')
ax3.plot( a_y, '--o', label='a_y', linewidth = 3)
ax3.plot( b_y, '-o', label='b_y', linewidth = 3)
ax3.plot( r_y, '--', label='r_y', linewidth = 3)
ax3.set_ylabel('[m]')
ax3.autoscale(enable=True, axis='x', tight=True)
ax3.legend()

ax4 = plt.subplot(5,2,4)
plt.title('Rising/falling edges in y axis')
ax4.plot( alpha_y, '--o', label='alpha_y', linewidth = 3)
ax4.plot( beta_y, '-o', label='beta_y', linewidth = 3)
ax4.set_ylim([-0.2,1.2])
ax4.set_ylabel('Binary')
ax4.autoscale(enable=True, axis='x', tight=True)
ax4.legend()

ax5 = plt.subplot(5,2,5)
plt.title('Support Phase')
ax5.plot( gamma, '-o', label='gamma', linewidth = 3)
ax5.set_ylim([-0.2,1.2])
ax5.set_ylabel('DS:1, SS:0')
ax5.autoscale(enable=True, axis='x', tight=True)
ax5.legend()

ax6 = plt.subplot(5,2,6)
plt.title('Potential Change from DS to SS')
ax6.plot( delta, '-o', label='delta', linewidth = 3)
ax6.set_ylim([-0.2,1.2])
ax6.set_ylabel('Binary')
ax6.autoscale(enable=True, axis='x', tight=True)
ax6.legend()

ax7 = plt.subplot(5,2,7)
plt.title('Previewed CoP')
ax7.plot( p_x, '-o', label='CoP_x', linewidth = 3)
ax7.plot( p_y,'-o',label='CoP_y', linewidth = 3)
ax7.set_ylabel('[m]')
ax7.autoscale(enable=True, axis='x', tight=True)
ax7.legend()

ax8 = plt.subplot(5,2,8)
plt.title('Previewed Jerk')
ax8.plot( u_x, '-o', label='u_x', linewidth = 3)
ax8.plot( u_y,'-o',label='u_y', linewidth = 3)
ax8.set_ylabel('')
ax8.autoscale(enable=True, axis='x', tight=True)
ax8.legend()
ax8.patch.set_facecolor('#ccccff')

# CoM State
ax9 = plt.subplot(5,2,9)
plt.title('Previewed CoM Position')
ax9.plot( h_x, '-o', label='h_x', linewidth = 3)
ax9.plot( h_y,'-o',label='h_y', linewidth = 3)
ax9.set_ylabel('[m]')
ax9.autoscale(enable=True, axis='x', tight=True)
ax9.legend()
ax9.patch.set_facecolor('#ccccff')

ax10 = plt.subplot(5,2,10)
plt.title('Previewed CoM Velocity')
ax10.plot( dh_x, '-o', label='dh_x', linewidth = 3)
ax10.plot( dh_y,'-o',label='dh_y', linewidth = 3)
ax10.set_ylabel('[m/s]')
ax10.autoscale(enable=True, axis='x', tight=True)
ax10.legend()
ax10.patch.set_facecolor('#ccccff')

# plotting
# plt.tight_layout()
plt.show()

answer = raw_input("Want to delete the data? [y/n] ")
if answer == 'y':
    os.remove(home + "solutionInPreview.txt")
    os.remove(home + "CoPinPreview.txt")
    os.remove(home + "BoSinPreview.txt")
    os.remove(home + "CoMinPreview.txt")

def functionname( plotTitle, values ):
   return;

