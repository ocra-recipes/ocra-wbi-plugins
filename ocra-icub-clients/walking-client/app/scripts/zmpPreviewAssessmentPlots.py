import numpy as np
import matplotlib.pyplot as plt

plt.style.use('ggplot')

home = "/home/jorhabib/Documents/octave/zmpPreviewController/"
currentComLinVel = np.loadtxt(home + "currentComLinVel.txt")
refLinVel = np.loadtxt(home + "refLinVel.txt");
intComPositionRef = np.loadtxt(home + "intComPositionRef.txt");
currentComPos = np.loadtxt(home + "currentComPos.txt");
currentZMP = np.loadtxt(home + "currentZMP.txt");
referenceZMP = np.loadtxt(home + "referenceZMP.txt");
comAcceleration = np.loadtxt(home + "comAcceleration.txt");
previewedZMP = np.loadtxt(home + "previewedZMP.txt");
optimalInput = np.loadtxt(home + "optimalInput.txt");

time = currentComPos[:,0]

# CoM velocity
# with plt.style.context('fivethirtyeight'):
ax1 = plt.subplot(4,1,1)
plt.title('ZMP Preview Controller Assessment')
ax1.plot(time, refLinVel[:,2], '--', label='Reference', linewidth = 3)
ax1.plot(time, currentComLinVel[:,2], label='Actual', linewidth = 3)
ax1.set_ylabel('CoM velocity Y axis [m/s]')
ax1.autoscale(enable=True, axis='x', tight=True)
ax1.legend()

ax2 = plt.subplot(4,1,2)
ax2.plot(time, intComPositionRef[:,2], '--', linewidth = 3, label='Reference')
ax2.plot(time, currentComPos[:,2], linewidth = 3, label='Actual')
ax2.set_ylabel('CoM Position [m]')
ax2.autoscale(enable=True, axis='x', tight=True)
ax2.legend()

ax3 = plt.subplot(4,1,3)
ax3.plot(time, referenceZMP[:,2], '--', linewidth = 3, label='Raw ZMP reference')
ax3.plot(time, previewedZMP[:,2], '--', linewidth = 3, label='Previewed ZMP reference')
ax3.plot(time, currentZMP[:,2], linewidth = 3, label='Actual ZMP')
ax3.plot(time, currentComPos[:,2], linewidth = 3, label='Actual CoM')
ax3.set_ylabel('Position [m]')
ax3.autoscale(enable=True, axis='x', tight=True)
ax3.legend()

ax4 = plt.subplot(4,1,4)
ax4.plot(time, optimalInput[:,2], '--', linewidth = 3, label='Optimal jerk')
ax4.set_ylabel('CoM Jerk');
ax4.set_xlabel('Time [s]')
ax4.autoscale(enable=True, axis='x', tight=True)
ax4.legend()

# plotting
# plt.tight_layout()
plt.show()
