#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, Range

pub_topic = ['/range_center', '/range_left', '/range_right']
sub_topic = ['/scan_center', '/scan_left', '/scan_right']
frame_ids = ['center_laser_link', 'left_laser_link', 'right_laser_link']
pub = []

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

    sonar_number = frame_ids.index(str(range_msg.header.frame_id))
    pub[sonar_number].publish(range_msg)

rospy.init_node('sonar_node', anonymous=True)
for topic in sub_topic:
    rospy.Subscriber(topic, LaserScan, callback)

for topic in pub_topic:
    pub.append(rospy.Publisher(topic, Range, queue_size=10))

rospy.spin()