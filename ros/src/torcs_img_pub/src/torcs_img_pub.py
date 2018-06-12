#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from torcs_msgs.msg import TORCSCtrl, TORCSSensors
from cv_bridge import CvBridge

import numpy as np
import mss

monitor = {'top': 40, 'left': 0, 'width': 800, 'height': 640}

def img_pub():
    pub = rospy.Publisher('/image_color', Image, queue_size=1)
    cbridge = CvBridge()
    rospy.init_node('torcs_ros_img_publisher', anonymous=True)
    rate = rospy.Rate(30) # Hz
    while not rospy.is_shutdown():
        with mss.mss() as sct:
            img = sct.grab(monitor)

        ros_img = cbridge.cv2_to_imgmsg(np.asarray(img), 'rgba8')
        pub.publish(ros_img)

        rate.sleep()


if __name__ == "__main__":
    try:
        img_pub()

    except rospy.ROSInterruptException:
        pass
