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


newimg_pub = rospy.Publisher('camera/color/image_new', Image, queue_size=10)

id_to_find = 72
marker_size = 20

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

horizontal_res = 640
vectical_res = 480

horizontal_fov = 62.2 * (math.pi / 180)
vectical_fov = 48.8 * (math.pi / 180)

found_count = 0
notfound_count = 0


dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
cam_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]

np_cam_matrix = np.array(cam_matrix)
np_dis_coeff = np.array(dist_coeff)


time_last = 0
time_to_wait = 0.1

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

                    marker_position = 'MARK POSITON: x='+x+' y= '+y+' z= '+z

                    aruco.drawDetectedMarkers(np_data,corners)
                    aruco.drawAxis(np_data, np_cam_matrix, np_dis_coeff, rvec, tvec, 10)
                    cv2.putText(np_data, marker_position, (10,50), 0, 0.5, (255, 0, 0), thickness = 1)

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
        main()
    except rospy.ROSInterruptException:
        pass