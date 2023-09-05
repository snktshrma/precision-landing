#!/usr/bin/env python
from pymavlink import mavutil
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import tf
import video_prec_gazebo
import aruco_calc
import logging

def signal_handler(signal, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


logging.basicConfig(level=logging.NOTSET)

the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))


class fcuModes:
    def _init_(self):
        pass

    def ackMsg(self):
    	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
		logger.info(msg)

    def setArm(self):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
		self.ackMsg()

	def armCheck(self):
		########## add pymavlink command to get arming status ##########
		pass

    def setDisarm(self):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)  # to force : 21196
		self.ackMsg()

    def setGuidedMode(self):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
		self.ackMsg()

	def setTakeoff(self,alt=3):
		the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
		self.ackMsg()

    def setRTL(self):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        self.ackMsg()

	def setAutoLandMode(self):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        self.ackMsg()

    def setPos(self,x=0,y=0,z=0):
    	the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
		        10,
		        the_connection.target_system, the_connection.target_component,
		        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b0000111111111000),
		        x, y, z,
		        0, 0, 0,
		        0, 0, 0, 0, 0
		    ))
    	logger.info(f"Sent {x},{y},{z} coordinates to FCU")

class Controller:
	def _init_(self):
		self.video = video_prec_gazebo.Cam_stream       # Get video frame
		self.frame = np.empty([], dtype=np.uint8)
		self.mode = fcuModes()

	def exec(self):
		self.frame = self.video.setup()
		if self.frame == 0:
			self.waitFun()
		else:
			self.landSeq(self.frame)

	def waitFun(self):
		logger.warning("Waiting to recieve the camera frame")

	def landSeq(self,frame):
		aruPos = aruco_calc.aru(frame)

def main():
	cnt = Controller()
	mode = fcuModes()

	########## add checks for arming to see if it is actually armed or not ##########
	mode.setArm()

	########## send few setpoints before shifting to guided mode ##########
	mode.setGuidedMode()

	########## Takeoff and confirm if takeoff is completed before going to the next step ##########


	########## Go to rough setpoint of the landing(aruco) marker ##########


	########## After reaching and getting stable, start aruco search and rest of the precision landing process ##########

	while True:
		cntr.exec()


########## Plan of action ##########
# 1. Arm and Takeoff in Guided mode (or Stabalised mode)
# 2. Then go to the setpoint
# 3. Then start aruco detection only if camera frames are recieved
# 4. Then calculate distance error and update that to fcu through local coordinates
# 5. Then after error is minimal enough, shift to land mode
