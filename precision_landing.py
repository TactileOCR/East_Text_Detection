#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array


vehicle = connect('tcp:127.0.0.1:5763', wait_ready= True, timeout= 60, heartbeat_timeout= 60)
vehicle.parameters['PLND_ENABLE']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30

print(vehicle.parameters)

velocity=0.5
takeoff_height=4


newimg_pub = rospy.Publisher('camera/color/image_new', Image, queue_size=10)

id_to_find = 72
marker_size = 20

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 720
vectical_res = 1280

horizontal_fov = 720 * (math.pi / 180)
vectical_fov = 1280 * (math.pi / 180)

found_count = 0
notfound_count = 0


dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
cam_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]

np_cam_matrix = np.array(cam_matrix)
np_dis_coeff = np.array(dist_coeff)


time_last = 0
time_to_wait = 0.1


def arm_and_takeoff(targetHeight):
    while vehicle.is_armable != True:
        print('Waiting for vehicle to become armable')
        time.sleep(1)
    print("Vehicle is now armable")

    vehicle.mode = 'GUIDED'

    while vehicle.mode != "GUIDED":
        print('Waiting for drone to enter GUIDED flight mode')
        time.sleep(1)
    print('GUIDED MODE ACTIVATED')

    vehicle.armed = True
    while vehicle.armed == False:
        print ("Waiting for vehicle to beomce armed")
        time.sleep(1)

    print("ARMED!")

    vehicle.simple_takeoff(targetHeight)

    while vehicle.location.global_relative_frame.alt < 0.95 * targetHeight:
        print('Current altitude: %d'%vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    
    print('Target Reached')

    return None

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b000111111000111,
        0,
        0,
        0,
        vx, 
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()







def call_back(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        np_data = rnp.numpify(message)
        gray_image  = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image= gray_image, dictionary=aruco_dict, parameters = parameters)

        try: 
            if ids is not None:
                if ids[0]==id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix = np_cam_matrix, distCoeffs = np_dis_coeff)
                    (rvec, tvec) =  (ret[0][0,0,:], ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                    y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

                    x_avg = x_sum / 4
                    y_avg = y_sum / 4

                    x_ang = (x_avg - horizontal_res * 0.5) * horizontal_fov / horizontal_res
                    y_ang = (y_avg - vectical_res * 0.5) * vectical_fov / vectical_res

                    if vehicle.mode != 'LAND':
                        vehicle.mode = 'LAND'
                        while vehicle.mode != 'LAND':
                            time.sleep(1)
                        print("Vehicle is in LAND mode")

                        send_land_message(x_ang,y_ang)
                    else:
                        send_land_message(x_ang, y_ang)

                    marker_position = 'MARK POSITON: x='+x+' y= '+y+' z= '+z

                    aruco.drawDetectedMarkers(np_data,corners)
                    aruco.drawAxis(np_data, np_cam_matrix, np_dis_coeff, rvec, tvec, 10)
                    cv2.putText(np_data, marker_position, (10,50), 0, 0.5, (255, 0, 0), thickness = 1)

                    print('FOUND COUNT: '+ str(found_count))
                    print('NOT FOUND COUNT: '+ str(notfound_count))

                    found_count += 1
                else:
                    notfound_count += 1
            else: 
                notfound_count += 1
        except Exception as e:
            notfound_count += 1
            print(e)
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None

def main():

    rospy.init_node('listener', anonymous= True)
    rospy.Subscriber("/camera/color/image_raw", Image, call_back)
    rospy.spin()
if __name__ == '__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        send_local_ned_velocity(velocity,0,0)
        time.sleep(1)
        main()
    except rospy.ROSInterruptException:
        pass