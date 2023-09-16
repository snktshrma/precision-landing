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
import threading
import signal
import time
import sys


takeoff_alt = 5
target_pos = [10,-9]
localPos = [0,0,0,0]
localVel = [0,0,0,0]
globalPos = [0,0,0,0,0]
check = 0
flag = 0

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
        logging.info(msg)

    def setArm(self):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.ackMsg()


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

    def setPos(self,x=0,y=0,z=-takeoff_alt):
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b0000111111111000),
            x, y, z,
            0, 0, 0,
            0, 0, 0, 0, 0
        ))
        logging.info(f"Sent {x},{y},{z} coordinates to FCU")

    def setPosLocal(self,x=0,y=0,z=takeoff_alt):
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,
            the_connection.target_system, the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b0000111111111000),
            x, y, z,
            0, 0, 0,
            0, 0, 0, 0, 0
        ))
        logging.info(f"Sent {x},{y},{z} coordinates to FCU")

    def recvGPS(self):
        global globalPos
        msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            #logging.info("GPS message recieved")
            globalPos[0] = msg.lat / 1e7
            globalPos[1] = msg.lon / 1e7
            globalPos[2] = msg.alt / 1000.0
            globalPos[3] = msg.relative_alt / 1000.0
            globalPos[4] = msg.time_boot_ms/1000.0
            return 1
        else:
            #logging.warning("GPS message not recieved yet")
            return 0


    def recvLocal(self):
        global localPos, localVel
        msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)

        if msg:
            #logging.info("Local message recieved")
            localPos[0] = msg.x
            localPos[1] = msg.y
            localPos[2] = msg.z
            localVel[0] = msg.vx
            localVel[1] = msg.vy
            return 1
        else:
            #logging.warning("Local message not recieved yet")
            return 0


class Controller:
    def __init__(self):
        self.video = video_prec_gazebo.Cam_stream()       # Get video frame
        self.frame = np.empty([], dtype=np.uint8)
        self.mode = fcuModes()

    def exec(self):
        self.frame = self.video.setup()
        if not self.video.frame_available():
            self.waitFun()
            return 0
        else:
            return aruco_calc.aru(self.frame)

    def waitFun(self):
        logging.warning("Waiting to recieve the camera frame")

def main():
    global check, flag
    cnt = Controller()
    mode = fcuModes()

    ########## send few setpoints before shifting to guided mode ##########
    mode.setGuidedMode()

    time.sleep(1)

    ########## send message recieving intervals for GPS and Local data ##########
    the_connection.mav.command_long_send(
                        the_connection.target_system, the_connection.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        33,
                        1e6 / 10,
                        0, 0, 0, 0,
                        0,
                )

    the_connection.mav.command_long_send(
                    the_connection.target_system, the_connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    32,
                    1e6 / 10,
                    0, 0, 0, 0,
                    0,
                )
    # t1 = threading.Thread(target=mode.setPos,args = (target_pos[0], target_pos[1],takeoff_alt,))
    # t2 = threading.Thread(target=mode.recvGPS)
    # t1.start()
    # t2.start()

    ########## add checks for arming to see if it is actually armed or not ##########
    while not the_connection.motors_armed():
        mode.setArm()
        time.sleep(0.5)

    ########## Takeoff and confirm if takeoff is completed before going to the next step ##########
    mode.setTakeoff(takeoff_alt)

    while True:
        mode.recvGPS()
        altErr = abs((globalPos[2]/100 - 1) - takeoff_alt)
        print(globalPos[2]/100, takeoff_alt)
        if altErr <= 0.2 and check == 0:
            time.sleep(3)
            check = 1
        elif altErr <= 0.2 and check == 1:
            logging.info("Takeoff completed")
            break
        else:
            check = 0



    ########## Go to rough setpoint of the landing(aruco) marker ##########
    # t1 = threading.Thread(target=mode.setPos,args = (target_pos[0], target_pos[1],takeoff_alt,))
    # t1.start()
    mode.setPos(target_pos[0], target_pos[1],-takeoff_alt)
    logging.info("Setpoint sent")

    ########################## add checks if drone reached the setpoint #######################
    # while True:
    #     mode.recvLocal()
    #     if abs(target_pos[0] - localPos[0]) <= 0.2 and abs(target_pos[1] - localPos[1]) <= 0.2:
    #         break




    ########## After reaching and getting stable, start aruco search and rest of the precision landing process ##########

    while True:
        mode.recvLocal()
        pos = cnt.exec()
        if not pos:
            continue
        else:
            if abs(target_pos[0] - localPos[0]) <= 0.2 and abs(target_pos[1] - localPos[1]) <= 0.2 and abs(localVel[0]) <= 0.1 and abs(localVel[1]) <= 0.1 and not flag:      ##### confirm if loop is necessary or a single iteration is accurate enough for getting x and y errors under 0
                
                time.sleep(3)
                flag = 1
                continue


                
                

            elif abs(target_pos[0] - localPos[0]) <= 0.2 and abs(target_pos[1] - localPos[1]) <= 0.2 and abs(localVel[0]) <= 0.1 and abs(localVel[1]) <= 0.1 and flag == 1:      ##### confirm if loop is necessary or a single iteration is accurate enough for getting x and y errors under 0
                mode.setPosLocal(pos[1],pos[0],0)
                print(pos)
                
                flag = 1
            elif abs(pos[0]) <= 0.2 and abs(pos[1]) <= 0.2:
                mode.setAutoLandMode()
                



main()


########## Plan of action ##########
# 1. Arm and Takeoff in Guided mode ----- Done
# 2. Then go to the setpoint ----- Done
# 3. Then start aruco detection only if camera frames are recieved ----- Done
# 4. Then calculate distance error and update that to fcu through local coordinates ----- Done
# 5. Then after error is minimal enough, shift to land mode ----- Done




############ Important features and additions ############
# add multithreading for recieving data over mavlink from AP ----- Done