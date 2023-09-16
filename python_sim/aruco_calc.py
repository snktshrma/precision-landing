import numpy as np
import cv2
import cv2.aruco as aruco
import math
import tf

# Global parameters
marker_pos_rel_camera = [ 0 , 0 , 0]
pos_camera = [0,0,0]
marker_size  = 58.5937500001 #- [cm]
img = np.empty([], dtype=np.uint8) # This will contain image frame from camera
ini_pt = [0,0,0]
set_id = 1


calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0 # 1.0
R_flip[1,1] = -1.0 # -1.0
R_flip[2,2] =-1.0 # -1.0

#--- Define the aruco dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)


#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def aru(frame):
    global pos_camera,l, ini_pt , marker_pos_rel_camera

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #-- Find all the aruco markers in the image
    corners, ids, rejected = detector.detectMarkers(gray)#, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    #print(ids)
    pos_camera = [0,0,0]

    if ids is not None and ids[0] == set_id:

        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Print the tag position in camera frame
        marker_pos_rel_camera[0] = tvec[0]/100
        marker_pos_rel_camera[1] = -tvec[1]/100
        marker_pos_rel_camera[2] = tvec[2]/100

        #-- Obtain the rotation matrix tag->camera
        R_ct= np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc= R_ct.T


        #-- Now get Position and attitude of the camera respect to the marker
        pos_camera = -R_tc*np.matrix(tvec).T
        aruco.drawDetectedMarkers(frame,corners)
        str_position = "Marker Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0],tvec[1],tvec[2])
        cv2.putText(frame, str_position, (0,100), font, 1, (0,255,0),2,cv2.LINE_AA)

        cv2.imshow('frame',frame)

        key = cv2.waitKey(1) & 0xFF
        return marker_pos_rel_camera