import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
print(sys.path)
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Import data points
data = np.loadtxt("laserCalibPoints.txt")

# Import camera intrinsics
f = cv2.FileStorage("calib.yaml",cv2.FILE_STORAGE_READ)
K0 = f.getNode("camera_matrix0").mat()
D0 = f.getNode("dist_coeffs0").mat()
K1 = f.getNode("camera_matrix1").mat()
D1 = f.getNode("dist_coeffs1").mat()

# Get triangulated 3D points
X = [ ptn[-3] for ptn in data ]
Y = [ ptn[-2] for ptn in data ]
Z = [ ptn[-1] for ptn in data ]

# Show 3D plot
#fig = plt.figure()
#ax = plt.axes(projection='3d')
#plt.scatter(X,Y,Z)
#plt.show()

objp = np.array([[ptn[-3],ptn[-2],ptn[-1]] for ptn in data])#np.vstack(np.meshgrid(X,Y,Z)).reshape(3,-1).T
objp = np.vstack(objp).reshape(3,-1).T
objp = objp.astype(np.float32)

objpoints = []
imgpoints0 = []
lasercmd = []

for ptn in data:
	objpoints.append(objp) # 3d point in real world space

	# Get image points in cameras and the corresponding laser command
	imgpoints0.append(np.array([ptn[0],ptn[1]]))
	#imgpoints0.reshape(2,-1).T

	# Remap from 0-4096 to (0-240,0-180)
	lasercmd.append(np.array([ptn[2],ptn[3]]))
	#lasercmd.reshape(2,-1).T

# Get the laser intrinsics
lasercmd = np.vstack(lasercmd).reshape(2,-1).T
ret, K, D, rvec0, tvec0 = cv2.calibrateCamera(objpoints, lasercmd, (240,180),None,None)
#
# # Get the extrinsics
# ret, _, _, _, _, R0, T0, E0, F0 = cv2.stereoCalibrate(objpoints,imgpoints0,imgpoints1, K0, D0, K, D, (240,180))
# ret, _, _, _, _, R1, T1, E1, F1 = cv2.stereoCalibrate(objpoints,imgpoints0,imgpoints1, K1, D1, K, D, (240,180))
#
# print('K: ', K)
# print('D: ', D)
#
# print('R0: ', R0)
# print('T0: ', T0)
# print('E0: ', E0)
# print('F0: ', F0)
#
# print('R1: ', R1)
# print('T1: ', T1)
# print('E1: ', E1)
# print('F1: ', F1)
#
# # Compute reprojection error
# tot_error = 0.
# for i in range(len(objpoints)):
# 	imgpointsRep0, _ = cv2.projectPoints(objpoints[i], rvec0[i], tvec0[i], K, D)
# 	error = cv2.norm(lasercmd[i],lasercmd[i], cv2.NORM_L2)/len(lasercmd)
# 	tot_error += error
#
# mean_error = tot_error/len(objpoints)
#
# print("Mean error: ", mean_error)

# Save calibration parameters in yaml