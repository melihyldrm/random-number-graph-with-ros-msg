#!/usr/bin/env python3

import rospy
from bitirme.msg import Coordinate
from std_msgs.msg import Header
import random

def coordinate_publisher():
    rospy.init_node('coordinate_publisher', anonymous=True)
    pub = rospy.Publisher('coordinates', Coordinate, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        coord_msg = Coordinate()
        
        #create time stamps
        header = Header()
        header.stamp = rospy.Time.now()  #current time
        header.frame_id = "base_link"
        coord_msg.header = header

        # create random coordinates
        coord_msg.x = random.uniform(-1, 1)
        coord_msg.y = random.uniform(-1, 1)
        coord_msg.z = random.uniform(-1, 1)

        rospy.loginfo("Publishing coordinates: x=%.2f, y=%.2f, z=%.2f", coord_msg.x, coord_msg.y, coord_msg.z)
        pub.publish(coord_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        coordinate_publisher()
    except rospy.ROSInterruptException:
        pass

