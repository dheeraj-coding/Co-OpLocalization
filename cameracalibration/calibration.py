# Referenced opencv documentation: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

import cv2
import numpy as np
import glob

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
_3dpts = []
_2dpts = []
objectp3d = np.zeros((1, 7 * 9, 3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:7, 0:9].T.reshape(-1, 2)
images = glob.glob('*.jpg')

for filename in images:
    image = cv2.imread(filename)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,9), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
 
    if ret:
        _3dpts.append(objectp3d)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        _2dpts.append(corners2)
        image = cv2.drawChessboardCorners(image, (7,9), corners2, ret)
 
    cv2.imshow('img', image)
    cv2.waitKey(1000)
 
cv2.destroyAllWindows()
 
ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
    _3dpts, _2dpts, gray.shape[::-1], None, None)
 
print("\n(Intrinsic Parameters)\nCamera matrix:\n" + str(matrix) + "\n\n(Extrinsic Parameters)\nRotation Vectors:\n" + str(r_vecs) + "\nTranslation Vectors:\n" + str(t_vecs))

'''
Output: 


(Intrinsic Parameters)
Camera matrix:
[[248.0855193    0.         120.61747803]
 [  0.         246.96378195 160.11072404]
 [  0.           0.           1.        ]]

(Extrinsic Parameters)
Rotation Vectors:
(array([[ 0.18112624],
       [ 0.35669671],
       [-3.10290574]]), array([[-0.14686075],
       [-0.24622082],
       [ 0.00180769]]), array([[ 0.00368973],
       [-0.3921042 ],
       [ 0.00299815]]))
Translation Vectors:
(array([[ 2.89297329],
       [ 4.43990507],
       [10.72201819]]), array([[-2.3808166],
       [-4.9729258],
       [10.4484131]]), array([[-2.19324425],
       [-4.19207007],
       [ 8.91397367]]))
       
'''