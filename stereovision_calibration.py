import numpy as np
import cv2 as cv
import glob



# These are the number of squares that are on the chessboard used for calibration
chessboardSize = (7, 11)
frameSize = (964, 686)



# This line sets the end of the algorithm, max 30 iterations OR 0.001 error
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)



# Object points are points in real 3D world (Example: ((0,0,0), (0,0,1), ..., (4, 5, 9), ...)
objectPoints = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objectPoints[:,:2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
objectPoints = objectPoints * 20



# These arrays are used to store object points and image points from all the images
# 3D points in real world space
objectPointsoints = []
# 2D points in image plane
imgpointsL = []
imgpointsR = []



# Takes all the images that have .png as extensions (left and right images of the stereocamera)
imagesLeft = glob.glob("images/stereoLeft/*.png")
print("Loading " + str(len(imagesLeft)) + " left images ...");
imagesRight = glob.glob("images/stereoRight/*.png")
print("Loading " + str(len(imagesLeft)) + " right images ...");



for imgLeft, imgRight in zip(imagesLeft, imagesRight):

	# Loads images and transform in black & white
	imgL = cv.imread(imgLeft)
	imgR = cv.imread(imgRight)
	grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
	grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)
	
	
	# Find the chessboard corners
	retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
	retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)
	
	
	# If found, add object points, image points (after refining them)
	if (retL and retR == True): 
	
		objectPointsoints.append(objectPoints)
		
		cornersL = cv.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
		imgpointsL.append(cornersL)
		
		cornersR = cv.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
		imgpointsR.append(cornersR)
		
		# Draw and display the corners
		cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
		cv.imshow('img left', imgL)
		cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
		cv.imshow('img right', imgR)
		cv.waitKey(1000)
		
		
cv.destroyAllWindows()





# This returns the camera matrix, distortion coefficients, rotation and translation vectors for both the left and right camera

retL, cameraMatrixL, distorsionCoefficientsL, rotationVectorsL, traslationVectorL = cv.calibrateCamera(objectPointsoints, imgpointsL, frameSize, None, None)
heigthL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distorsionCoefficientsL, (widthL, heigthL), 1, (widthL, heigthL))

retR, cameraMatrixR, distorsionCoefficientsR, rotationVectorsR, traslationVectorR = cv.calibrateCamera(objectPointsoints, imgpointsR, frameSize, None, None)
heigthR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distorsionCoefficientsR, (widthR, heigthR), 1, (widthR, heigthR))



# This piece of code performs the stereo calibration

flags = 0
flags |= cv.CALIB_FIX_INTRINSIC
criteria_stereo = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


retStereo, newCameraMatrixL, distorsionCoefficientsL, newCameraMatrixR, distorsionCoefficientsR, rot ,trans ,essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objectPointsoints, imgpointsL, imgpointsR, newCameraMatrixL, distorsionCoefficientsL, newCameraMatrixR, distorsionCoefficientsR, grayL.shape [:: -1], criteria_stereo, flags)

rectifyScale = 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv.stereoRectify(newCameraMatrixL, distorsionCoefficientsL, newCameraMatrixR, distorsionCoefficientsR, grayL.shape[::-1], rot, trans, rectifyScale, (0,0))

stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distorsionCoefficientsL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distorsionCoefficientsR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)



print("Saving parameters...")

cv_file = cv.FileStorage('stereoMap.xml', cv.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x', stereoMapL[0])
cv_file.write('stereoMapL_y', stereoMapL[1])

cv_file.write('stereoMapR_x', stereoMapR[0])
cv_file.write('stereoMapR_y', stereoMapR[1])

cv_file.release()
