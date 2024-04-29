#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Image

def callback(data):


    rospy.loginfo("Here's what was sbuscribed: %s", data.data)
    print('Data from /topic_name received')

def main():

    rospy.init_node('listener', anonymous= True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass