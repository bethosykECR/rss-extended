import matplotlib.pyplot as plt
import csv
import os
import sys
from mpl_toolkits import mplot3d
import scipy.linalg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm

################################################
# PARAMETERS (set it up YO-self)
################################################
FOLDER_PATH = '/home/user/alena/rgt/res/res-int'
#
search_name_x = 'lon accel max'
search_name_y = 'response time'
#
step = 0.1
x_min = 0.0
x_max = 10.0
y_min = 0.05
y_max = 5.0
#
flag_plot_surface = False
fontsize =15
################################################

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


X1, X2 = getData2D(os.path.join(FOLDER_PATH, 'x_history.csv'))
R = getData1D(os.path.join(FOLDER_PATH, 'f_history.csv'))
################
# filter part
################
'''
I = [i for i, r in enumerate(R) if r > 18]
for index in sorted(I, reverse=True): # reverse order for not missing shifted indices
    del R[index]
    del X1[index]
    del X2[index]
'''
data = np.column_stack((X1,X2,R))
X_mesh,Y_mesh = np.meshgrid(np.arange(x_min, x_max, step), np.arange(y_min, y_max, step))


ax = plt.axes(projection='3d')
if flag_plot_surface:
	XX = X_mesh.flatten()
	YY = Y_mesh.flatten()

	A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
	C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])
	Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X_mesh.shape)
	ax.plot_surface(X_mesh, Y_mesh, Z, rstride=1, cstride=1, alpha=0.8, cmap=cm.RdYlGn, linewidth=0, antialiased=False)

ax.plot_surface(X_mesh, Y_mesh, X_mesh*0, rstride=1, cstride=1, alpha=0.1, cmap=cm.RdYlGn, linewidth=0, antialiased=False)
ax.scatter3D(X1, X2, R, c=R, cmap=cm.RdYlGn, alpha=0.8)
ax.set_xlabel(search_name_x, fontsize=fontsize)
ax.set_ylabel(search_name_y, fontsize=fontsize)
ax.set_zlabel('Robustness', fontsize=fontsize)
plt.tick_params(labelsize=10)
plt.xlim(x_min, x_max)     
plt.ylim(y_min, y_max)     
plt.show()
