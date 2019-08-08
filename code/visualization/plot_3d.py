import matplotlib.pyplot as plt
import csv
import os
import sys
from mpl_toolkits import mplot3d
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm

def getData1D(filepath):
	file = open(filepath)
	reader = csv.reader(file)
	data_str = list(reader)
	file.close()
	X = [float(x[0]) for x in data_str]
	return X

def getData2D(filepath):
	file = open(filepath)
	reader = csv.reader(file)
	data_str = list(reader)
	file.close()
	X1 = [float(x[0]) for x in data_str]
	X2 = [float(x[1]) for x in data_str]
	return X1, X2

FOLDER_PATH = '/home/user/alena/rgt/res/res-3d_plot2'
X1, X2 = getData2D(os.path.join(FOLDER_PATH, 'x_history.csv'))
R = getData1D(os.path.join(FOLDER_PATH, 'f_history.csv'))

################
# filter part
################
I = [i for i, r in enumerate(R) if r > 18]
for index in sorted(I, reverse=True): # reverse order for not missing shifted indices
    del R[index]
    del X1[index]
    del X2[index]

data = np.column_stack((X1,X2,R))
step = 0.1
X,Y = np.meshgrid(np.arange(0.0, 10.0, step), np.arange(0.01, 5.0, step))
XX = X.flatten()
YY = Y.flatten()

A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])
Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)



ax = plt.axes(projection='3d')
ax.scatter3D(X1, X2, R, c=R, cmap=cm.RdYlGn, alpha=0.8)
#ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.8, cmap=cm.RdYlGn, linewidth=0, antialiased=False)
ax.plot_surface(X, Y, X*0, rstride=1, cstride=1, alpha=0.1, cmap=cm.RdYlGn, linewidth=0, antialiased=False)
ax.set_xlabel('lon_brake_max', fontsize=15)
ax.set_ylabel('lon_accel_max', fontsize=15)
ax.set_zlabel('Robustness', fontsize=15)
plt.tick_params(labelsize=10)
plt.show()
