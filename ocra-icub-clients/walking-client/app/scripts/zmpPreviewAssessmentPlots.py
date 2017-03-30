import numpy as np
import matplotlib.pyplot as plt
import os

plt.style.use('ggplot')

home = "/home/ryan/octave/zmpPreviewController/"
# currentComLinAcc = np.loadtxt(home + "currentComLinAcc.txt")
# refComLinAcc = np.loadtxt(home + "refComLinAcc.txt");
intComPositionRef = np.loadtxt(home + "intComPositionRef.txt");
currentComPos = np.loadtxt(home + "currentComPos.txt");
currentZMP = np.loadtxt(home + "currentZMP.txt");
referenceZMP = np.loadtxt(home + "referenceZMP.txt");
previewedZMP = np.loadtxt(home + "previewedZMP.txt");
optimalInput = np.loadtxt(home + "optimalInput.txt");
# currentRightFootPosition = np.loadtxt(home + "currentRightFootPosition.txt");

time = currentComPos[:,0]

# CoM velocity
# with plt.style.context('fivethirtyeight'):
ax1 = plt.subplot(4,1,1)
plt.title('ZMP Preview Controller Assessment')
# ax1.plot(time, refComLinAcc[:,2], '--', label='Reference', linewidth = 3)
# ax1.plot(time, currentComLinAcc[:,2], label='Actual', linewidth = 3)
ax1.set_ylabel('CoM acceleration Y axis [m/s^2]')
ax1.autoscale(enable=True, axis='x', tight=True)
ax1.legend()

ax2 = plt.subplot(4,1,2)
ax2.plot(time, referenceZMP[:,2], '--', linewidth = 3, label='Raw ZMP reference')
ax2.plot(time, previewedZMP[:,2], '--', linewidth = 3, label='Previewed ZMP reference')
ax2.plot(time, currentZMP[:,2], linewidth = 3, label='Actual ZMP')
ax2.plot(time, currentComPos[:,2], linewidth = 3, label='Actual CoM')
ax2.set_ylabel('Position [m]')
ax2.autoscale(enable=True, axis='x', tight=True)
ax2.legend()

ax3 = plt.subplot(4,1,3)
ax3.plot(time, optimalInput[:,2], '--', linewidth = 3, label='Optimal jerk')
ax3.set_ylabel('CoM Jerk');
ax3.set_xlabel('Time [s]')
ax3.autoscale(enable=True, axis='x', tight=True)
ax3.legend()

# ax4 = plt.subplot(4,1,4)
# ax4.plot(time, currentRightFootPosition[:,3], '--', linewidth = 3, label='Right Foot vertical position')
# ax4.set_ylabel('Position [m]')
# ax4.set_xlabel('Time [s]')
# ax4.autoscale(enable=True, axis='x', tight=True)
# ax4.legend()

# plotting
# plt.tight_layout()
plt.show()

# os.remove(home + "currentComLinAcc.txt")
# os.remove(home + "refComLinAcc.txt");
os.remove(home + "intComPositionRef.txt");
os.remove(home + "currentComPos.txt");
os.remove(home + "currentZMP.txt");
os.remove(home + "referenceZMP.txt");
os.remove(home + "previewedZMP.txt");
os.remove(home + "optimalInput.txt");
# os.remove(home + "currentRightFootPosition.txt");
