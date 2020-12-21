import numpy as np
import matplotlib.pyplot as plt

results = np.loadtxt(r'C:\Users\swan11jf\Google Drive\Cambridge\Engineering\mars_lander\C++ Tasks\spring_euler\cmake-build-debug\euler.txt')
results2 = np.loadtxt(r'C:\Users\swan11jf\Google Drive\Cambridge\Engineering\mars_lander\C++ Tasks\spring_verlet\cmake-build-debug\verlet.txt')

plt.figure()
plt.title('EULER')
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.legend()
plt.show()

plt.clf()

plt.figure()
plt.title('VERLET')
plt.xlabel('time (s)')
plt.grid()
plt.plot(results2[:, 0], results2[:, 1], label='x (m)')
plt.plot(results2[:, 0], results2[:, 2], label='v (m/s)')
plt.legend()
plt.show()

