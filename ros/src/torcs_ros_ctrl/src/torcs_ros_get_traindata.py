#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from torcs_msgs.msg import TORCSCtrl, TORCSSensors
from cv_bridge import CvBridge
import message_filters

import numpy as np
import csv
import PIL.Image as pimg


def img_sub_callback(image, torcssensors):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image, 'bgra8')
    steer = torcssensors.angle
    speed_x = torcssensors.speedX

    rospy.loginfo("steer: {}, speed_x: {}".format(steer, speed_x))

def get_train_data():
    rospy.init_node('torcs_ros_train_data_subscriber', anonymous=True)

    img_sub = message_filters.Subscriber('image_color', Image)
    veh_sub = message_filters.Subscriber('torcs_sensors', TORCSSensors)

    # ts = message_filters.TimeSynchronizer([img_sub, veh_sub], 10)
    ts = message_filters.ApproximateTimeSynchronizer([img_sub, veh_sub], 10, 0.1, allow_headerless=False)
    ts.registerCallback(img_sub_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        get_train_data()

    except rospy.ROSInterruptException:
        pass
