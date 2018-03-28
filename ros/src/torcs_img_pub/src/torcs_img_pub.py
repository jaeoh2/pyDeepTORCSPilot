#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 20Hz
    while not rospy.is_shutdown():
        hell_str = "hell world {}".format(rospy.get_time())
        rospy.loginfo(hell_str)
        pub.publish(hell_str)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
