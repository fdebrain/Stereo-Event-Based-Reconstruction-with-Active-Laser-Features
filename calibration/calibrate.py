import numpy as np
import sys
#sys.path.remove('')
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
print(sys.path)
import cv2

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# World points
X = [ i for i in range(0,8*3,3) ]
Y = [ i for i in range(0,5*3,3) ]
Z = 0
objp = np.vstack(np.meshgrid(X,Y,Z)).reshape(3,-1).T
objp = objp.astype(np.float32)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints0 = [] # 2d points in image plane.
imgpoints1 = [] # 2d points in image plane.


davis0 = cv2.VideoCapture('data/calibChess0.avi')
davis1 = cv2.VideoCapture('data/calibChess1.avi')

ret0, frame0 = davis0.read()
ret1, frame1 = davis1.read()
cnt = 0

while(ret0==True and ret1==True):

	ret0, frame0 = davis0.read()
	ret1, frame1 = davis1.read()

	cnt += 1
	if cnt%20!=0:
		continue

	gray0 = cv2.cvtColor(frame0,cv2.COLOR_BGR2GRAY)
	gray1 = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)

	# Find the chess board corners
	found0, corners0 = cv2.findChessboardCorners(gray0, (8,5), None)
	found1, corners1 = cv2.findChessboardCorners(gray1, (8,5), None)

	# If found, add object points, image points (after refining them)
	if found0==True and found1==True:
		objpoints.append(objp)
		cornersRef0 = cv2.cornerSubPix(gray0, corners0, (5,5), (-1,-1), criteria) # (11,11)
		cornersRef1 = cv2.cornerSubPix(gray1, corners1, (5,5), (-1, -1), criteria)
		imgpoints0.append(cornersRef0)
		imgpoints1.append(cornersRef1)

		# Draw and display the corners
		frame0 = cv2.drawChessboardCorners(frame0, (8,5), cornersRef0, found0)
		frame1 = cv2.drawChessboardCorners(frame1, (8,5), cornersRef1, found1)

		cv2.imshow('frame0', frame0)
		cv2.imshow('frame1', frame1)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

davis0.release()
davis1.release()
cv2.destroyAllWindows()

ret, K0, D0, rvec0, tvec0 = cv2.calibrateCamera(objpoints, imgpoints0, (240,180),None,None)
ret, K1, D1, rvec1, tvec1 = cv2.calibrateCamera(objpoints, imgpoints1, (240,180),None,None)

ret, K0, D0, K1, D1, R, T, E, F = cv2.stereoCalibrate(objpoints,imgpoints0,imgpoints1, K0, D0, K1, D1, (240,180))

print('K0: ', K0)
print('D0: ', D0)
print('K1: ', K1)
print('D1: ', D1)
print('R: ', R)
print('T: ', T)
print('E: ', E)
print('F: ', F)

# Compute reprojection error
tot_error0 = 0.
tot_error1 = 0.
for i in range(len(objpoints)):
	imgpointsRep0, _ = cv2.projectPoints(objpoints[i], rvec0[i], tvec0[i], K0, D0)
	imgpointsRep1, _ = cv2.projectPoints(objpoints[i], rvec1[i], tvec1[i], K1, D1)

	error0 = cv2.norm(imgpoints0[i],imgpointsRep0, cv2.NORM_L2)/len(imgpointsRep0)
	error1 = cv2.norm(imgpoints1[i], imgpointsRep1, cv2.NORM_L2) / len(imgpointsRep1)
	tot_error0 += error0
	tot_error1 += error1

mean_error0 = tot_error0/len(objpoints)
mean_error1 = tot_error1/len(objpoints)

print("Mean error0: ", mean_error0)
print("Mean error1: ", mean_error1)

# Save calibration parameters in JSON
import yaml
fname = "calib.yaml"
save = cv2.FileStorage(fname, cv2.FileStorage_WRITE)

save.write('camera_matrix0',K0)
save.write('dist_coeffs0',D0)
save.write('camera_matrix1',K1)
save.write('dist_coeffs1',D1)
save.write('R',R)
save.write('T',T)
save.write('E',E)
save.write('F',F)
save.release()

#data = { 'camera_matrix0':K0.tolist(), 'dist_coeffs0':D0.tolist(),
#          'camera_matrix1':K1.tolist(), 'dist_coeffs1':D1.tolist(),
#          'R':R.tolist(), 'T':T.tolist(), 'E':E.tolist(), 'F':F.tolist(),
#          'error0':mean_error0,
#          'error1':mean_error1 }
#
# with open(fname, "w") as f:
# 	yaml.dump(data, f,default_flow_style=False)#indent=4)