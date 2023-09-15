#!/usr/bin/env python
from pymavlink import mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import tf
import video_prec_gazebo
import aruco_calc

# Global parameters
marker_pos_rel_camera = [ 0 , 0 , 0]
landing_station_velocity = [0,0,0]
orien = [0,0,0,0]
landing_station_coordinate = [0 , 0 , 0]
pos_camera = [0,0,0]
flag = 0
marker_size  = 58.5937500001 #- [cm]
img = np.empty([], dtype=np.uint8) # This will contain image frame from camera
ini_pt = [0,0,0]
height_global = 5
set_id = 1

previous_distance_x = 0
previous_distance_y = 0

calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0 # 1.0
R_flip[1,1] = -1.0 # -1.0
R_flip[2,2] =-1.0 # -1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

class fcuModes:
    def _init_(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s"%e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set."%e)

def sign(input):
    if input < 0:
        return -1
    return 1

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        self.drone_orientation_euler = [ 0 , 0 , 0]
        self.euler = [0,0,0]
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
     
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        
        self.model_name = "landing_station"        
        
        self.vtoorienelocity = [0.0, 0.0, 0.0]

        # A Message for the current local position of the drone
        self.local_pos = Point()

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 0.0
        self.sp.yaw = 1.57
        
        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink

    def landing_pad_gps_pos_cb(self,msg):
        global current_time , previous_time ,landing_station_velocity ,landing_station_coordinate
        landing_station_coordinate[0] =  111227.13608228297 * (msg.latitude - 49.9)
        landing_station_coordinate[1] = -71844.35659053615 * (msg.longitude - 8.9)
        landing_station_coordinate[2] = msg.altitude
        rospy.sleep(1)


    # Callbacks
    
    # callback for getting landing station velocity
    def landing_station_velocity_cb(self,msg):
        landing_station_velocity[0] = msg.vector.x
        landing_station_velocity[1] = msg.vector.y
    
    # callback for getting vtol orientation
    def vtol_orientation_cb(self,msg):
        drone_orientation_quaternion = [0 , 0 , 0 , 0]
        drone_orientation_quaternion[0] = msg.orientation.x
        drone_orientation_quaternion[1] = msg.orientation.y
        drone_orientation_quaternion[2] = msg.orientation.z
        drone_orientation_quaternion[3] = msg.orientation.w
        
        # converting quaternion orientation to eular
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = \
        tf.transformations.euler_from_quaternion([drone_orientation_quaternion[0], \
        drone_orientation_quaternion[1], drone_orientation_quaternion[2], \
        drone_orientation_quaternion[3]])

    ## vtol's local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## callback for getting vtol's State parameters
    def stateCb(self, msg):
        self.state = msg
        
    # callback for current gazebo model state parmas.
    def modelStatesCallback(self,msg):
        global t, ini_pt, orien
        index_of_interest = -1
        for i in range(len(msg.name)):
            if msg.name[i] == self.model_name:
                index_of_interest = i
                break
        if index_of_interest >= 0:
            model_state = ModelState()
            model_state.model_name = self.model_name
            model_state.pose = msg.pose[index_of_interest]
            landing_station_coordinate[0] = model_state.pose.position.x
            landing_station_coordinate[1] = model_state.pose.position.y
            landing_station_coordinate[2] = model_state.pose.position.z
            orien[0] = model_state.pose.orientation.x
            orien[1] = model_state.pose.orientation.y
            orien[2] = model_state.pose.orientation.z
            orien[3] = model_state.pose.orientation.w
            #print(orien)
            (self.euler[0], self.euler[1], self.euler[2]) = \
            tf.transformations.euler_from_quaternion([orien[0], \
            orien[1], orien[2], \
            orien[3]])

    # camera image callback
    def image_callback(self,data):
        global t, height_global, orien, set_id
        self.sp.position.z = height_global
        if self.local_pos.z > 1.2 and set_id == 1:
            print("I am here")
            try:
                img = bridge.imgmsg_to_cv2(data, "bgr8")
                ids = aru(img)
                self.sp.position.z = height_global
                if ids is not None:
                    
                    distance_x = 5*math.tan(-self.drone_orientation_euler[0])+marker_pos_rel_camera[0]/math.cos(-self.drone_orientation_euler[0])
                    distance_y = 5*math.tan(-self.drone_orientation_euler[1])+marker_pos_rel_camera[1]/math.cos(-self.drone_orientation_euler[1])
                    self.sp.position.x =  self.local_pos.x+ distance_x  
                    self.sp.position.y =  self.local_pos.y+ distance_y 
                    self.sp.velocity.x = sign(landing_station_velocity[0])*1 
                    self.sp.velocity.y = sign(landing_station_velocity[1])*1 
                    self.sp.yaw = 1.57 + self.euler[2]

                    if ids is not None and -0.3 <= (self.local_pos.x - self.sp.position.x) <= 0.3 and -0.3 <= (self.local_pos.y - self.sp.position.y) <= 0.3:
                        set_id = 2
                        self.sp.position.z = 1
                        height_global = 1
                        self.sp.yaw = 1.57 + self.euler[2]

                else:
                    self.sp.position.x = landing_station_coordinate[0]
                    self.sp.position.y = landing_station_coordinate[1]
                    self.sp.velocity.x = sign(landing_station_velocity[0])*0.1 
                    self.sp.velocity.y = sign(landing_station_velocity[1])*0.1
                    self.sp.yaw = 1.57
                    self.sp.position.z = height_global
                    print("Using global")

            except CvBridgeError as e:
                return
    # front-camera image callback
    def image_sec(self,data):
        global t, height_global, orien, set_id, previous_distance_x, previous_distance_y
        try:
            img = bridge.imgmsg_to_cv2(data, "bgr8")
            ids = aru(img)
            
            if ids is not None and set_id == 2:

                self.sp.position.x =  self.local_pos.x + marker_pos_rel_camera[0]  
                self.sp.position.y =  self.local_pos.y + marker_pos_rel_camera[2] - 5.85

                self.sp.velocity.x = (landing_station_velocity[0]) + 1.2*(marker_pos_rel_camera[0] - previous_distance_x)#orien[0] + 1
                self.sp.velocity.y = (landing_station_velocity[1]) + 1.2*(marker_pos_rel_camera[2] - previous_distance_y)

                previous_distance_x = marker_pos_rel_camera[0]
                previous_distance_y = marker_pos_rel_camera[2]
                self.sp.yaw = 1.57 + self.euler[2]


                if ids is not None and set_id == 2 and -0.2 <= (marker_pos_rel_camera[0]) <= 0.2 and -0.2 <= (marker_pos_rel_camera[2] - 5.85) <= 0.2:
                    self.sp.velocity.z = 1
                    self.sp.position.z -= 0.1
                    height_global -= 0.1
                    self.sp.yaw = 1.57 + self.euler[2]
            
        except CvBridgeError as e:
            return
#_____________________________________Distance calculation from aruco______________________________________________


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
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)#, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
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
        return(ids)
    

    #___________________________________________________________________________________
    # Main function
def main():

    # initiate node
    rospy.init_node('final', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    rospy.Subscriber("gazebo/model_states", ModelStates, cnt.modelStatesCallback)

    rospy.Subscriber("/standard_vtol/camera/image_raw", Image, cnt.image_callback)
    rospy.Subscriber("/standard_vtol/camera_sec/image_raw_sec", Image, cnt.image_sec)
    rospy.Subscriber("/mavros/imu/data", Imu, cnt.vtol_orientation_cb)
    rospy.Subscriber("/gps_velocity", Vector3Stamped,cnt.landing_station_velocity_cb)
    rospy.Subscriber('/gps', NavSatFix , cnt.landing_pad_gps_pos_cb )


    # Setpoint publisher
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        sp_pub.publish(cnt.sp)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass