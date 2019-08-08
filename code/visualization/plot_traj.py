import matplotlib.pyplot as plt
import csv
import os
import sys
from mpl_toolkits import mplot3d

def getData3D(filepath):
	file = open(filepath)
	reader = csv.reader(file)
	data_str = list(reader)
	file.close()
	X1 = [float(x[0]) for x in data_str[:-1]]
	X2 = [float(x[1]) for x in data_str[:-1]]
	X3 = [float(x[2]) for x in data_str[:-1]]
	return X1, X2, X3

#FOLDER_PATH = '/home/user/alena/rgt/res/res-default'
FOLDER_PATH = '/home/user/alena/rgt/res/res-collision'
D, V_ego, V_pov = getData3D(os.path.join(FOLDER_PATH, 'trajectory0.csv'))
I = range(len(D))

plt.plot(I, D, 'r-', linewidth=6) 
plt.plot(I, V_ego, 'g-', linewidth=2) 
plt.plot(I, V_pov, 'b-', linewidth=2)
#plt.patch.set_visible(False) 
plt.xlabel('Time [frame]', fontsize=20)
plt.ylabel('Magnitude', fontsize=20)
plt.tick_params(labelsize=20)
plt.tick_params(top='off', bottom='off', left='off', right='off', labelleft='on', labelbottom='on')
#plt.legend(['Min Distance','V_ego', 'V_pov'], loc='best')
plt.show()