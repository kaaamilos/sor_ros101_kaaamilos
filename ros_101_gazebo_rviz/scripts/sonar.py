#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, Range

pub_topic = '/range'
sub_topic = '/scan'
pub = rospy.Publisher(pub_topic, Range, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.ranges[0])
    range_msg = Range()
    range_msg.header.stamp = rospy.Time.now()
    range_msg.header.frame_id = data.header.frame_id
    range_msg.radiation_type = Range.ULTRASOUND
    range_msg.field_of_view = 0.0698132
    range_msg.max_range = 30.0
    range_msg.min_range = 0.1
    range_msg.range = sum(data.ranges)/len(data.ranges)
    pub.publish(range_msg)

rospy.init_node('sonar_node', anonymous=True)
rospy.Subscriber(sub_topic, LaserScan, callback)

rospy.spin()