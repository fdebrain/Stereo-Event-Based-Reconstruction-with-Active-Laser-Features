import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
print(sys.path)
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 0.001)

# Import data points
data = np.loadtxt("laserCalibPoints.txt")

# Import camera intrinsics
f = cv2.FileStorage("calib.yaml",cv2.FILE_STORAGE_READ)
K0 = f.getNode("camera_matrix0").mat()
D0 = f.getNode("dist_coeffs0").mat()
K1 = f.getNode("camera_matrix1").mat()
D1 = f.getNode("dist_coeffs1").mat()

# Get triangulated 3D points
# X = [ ptn[-3] for ptn in data ]
# Y = [ ptn[-2] for ptn in data ]
# Z = [ ptn[-1] for ptn in data ]

# Show 3D plot
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# plt.scatter(X,Y,Z)
# plt.show()

# Get laser coordinates (x,y) and remap to [180,240]
def remap(x,y):
	x_min = 1000.
	x_max = 3500.
	y_min = 500.
	y_max = 3500.
	return x,y
	#return [240.*(float(x)-x_min)/(x_max-x_min), 180.*(float(y)-y_min)/(y_max-y_min)]

# Get point coordinates in camera 1 (x0,y0)
imgpoints0 = np.array([[ptn[1],ptn[0]] for ptn in data])
imgpoints0 = np.reshape(imgpoints0.astype(np.float32),(1,-1,1,2))

# Get point coordinates in camera 1 (x0,y0)
imgpoints1 = np.array([[ptn[3],ptn[2]] for ptn in data])
imgpoints1 = np.reshape(imgpoints1.astype(np.float32),(1,-1,1,2))

laserptn = np.array([ remap(ptn[5],ptn[4]) for ptn in data])
lasercmd = np.reshape(laserptn.astype(np.float32),(1,-1,1,2))

# Get 3D point coordinates (X,Y,Z)
objp = np.array([[ptn[6],ptn[7],0] for ptn in data])
objpoints = np.reshape(objp.astype(np.float32),(1,-1,1,3))

# Show 2D plot
# fig2 = plt.figure()
# ax2 = plt.axes()
# plt.scatter([ptn[0] for ptn in laserptn],[ptn[1] for ptn in laserptn])
# plt.scatter([ptn[0] for ptn in data],[ptn[1] for ptn in data],c='r')
# plt.show()

# Get the laser intrinsics
ret, K, D, rvec, tvec = cv2.calibrateCamera(objpoints, lasercmd, (3000,3000),None,None)
print('K: ', K)
print('D: ', D)

# Get the extrinsics
rms1, _, _, _, _, R0, T0, E0, F0 = cv2.stereoCalibrate(objpoints,lasercmd,imgpoints0, K0, D0, K, D, (240,180),
                                                       flags=cv2.CALIB_FIX_INTRINSIC,criteria=criteria)
rms2, _, _, _, _, R1, T1, E1, F1 = cv2.stereoCalibrate(objpoints,lasercmd,imgpoints1, K1, D1, K, D, (240,180),
                                                       flags=cv2.CALIB_FIX_INTRINSIC, criteria=criteria)
print('R0: ', R0)
print('T0: ', T0)
print('E0: ', E0)
print('F0: ', F0)
print('R1: ', R1)
print('T1: ', T1)
print('E1: ', E1)
print('F1: ', F1)
print('rms1: ',rms1/len(objpoints[0]))
print('rms2: ',rms2/len(objpoints[0]))

# Compute projection error
imgpointsRep, _ = cv2.projectPoints(objpoints[0], rvec[0], tvec[0], K, D)
error = cv2.norm(lasercmd[0],imgpointsRep, cv2.NORM_L2)/len(imgpointsRep)
print("Error: ", error)

# Show projection of object points and true laser commands
fig3 = plt.figure()
ax3 = plt.axes()
plt.title("Reprojection of 3D points into 2D laser command")
plt.scatter([ptn[0][0] for ptn in imgpointsRep],[ptn[0][1] for ptn in imgpointsRep],c='blue')
plt.scatter([ptn[0][0] for ptn in lasercmd[0]],[ptn[0][1] for ptn in lasercmd[0]],c='red')
plt.legend(["Projected 3D points", "Laser command"])
plt.show()

# Save calibration parameters in YAML
import yaml
fname = "laserCalib.yaml"
save = cv2.FileStorage(fname, cv2.FileStorage_WRITE)

save.write('camera_matrix_laser',K)
save.write('dist_coeffs_laser',D)
save.write('R0',R0)
save.write('T0',T0)
save.write('E0',E0)
save.write('F0',F0)
save.write('R1',R1)
save.write('T1',T1)
save.write('E1',E1)
save.write('F1',F1)
save.release()
