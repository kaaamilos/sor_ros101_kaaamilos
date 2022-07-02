import time
import math
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

ROTATE_THRESHOLD = 0.05
POSE_THRESHOLD = 0.1

class Asg03Navigation():
    def __init__(self):
        self.target_x = 0
        self.target_y = 0
        self.target_angle = 0
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.speed = 2
        self.max_speed = 2

        self.msg = Twist()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_callback)
        self.step = 'waiting'
        self.last_log = 0
        self.linear_speed = 0

    def __goal_callback(self, goal):
        self.target_x = goal.pose.position.x
        self.target_y = goal.pose.position.y
        if self.step == 'waiting':
            self.xy_to_angle()
            self.step = 'init'

    def xy_to_angle(self):
        self.target_angle = math.atan2(self.target_y, self.target_x)
        rospy.loginfo(f'calculated_angle: {self.target_angle}')

    def calculate_distance(self):
        goal_position = math.sqrt(self.target_x**2 + self.target_y**2)
        current_position = math.sqrt(self.x**2 + self.y**2)
        self.linear_speed = (goal_position - current_position) * self.speed
        self.linear_speed = self.linear_speed if self.linear_speed <= self.max_speed else self.max_speed
        rospy.logdebug(f'linear speed: {self.linear_speed}')

    def log_position(self):
        now = time.time()
        if now > self.last_log + 1 and self.step != 'end':
            self.last_log = time.time()

    def update_pose(self):
        (trans, rot) = self.listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
        self.x = trans[0]
        self.y = trans[1]
        (_, _, self.yaw) = euler_from_quaternion(rot)

    def reach_goal(self):
        if self.step == 'init':
            if self.target_angle > 0:
                current_speed = self.target_angle - self.yaw
                current_speed = current_speed if current_speed <= self.max_speed else self.max_speed
                self.msg.angular.z = current_speed * self.speed
            else:
                current_speed = self.yaw - self.target_angle
                current_speed = current_speed if current_speed <= self.max_speed else self.max_speed
                self.msg.angular.z = -1 * (current_speed * self.speed)

            if current_speed < ROTATE_THRESHOLD:
                self.msg.angular.z = 0
                self.step = 'reach_goal'

        elif self.step == 'reach_goal':
            self.calculate_distance()
            if self.linear_speed > POSE_THRESHOLD:
                self.msg.linear.x = self.linear_speed
            else:
                self.msg.linear.x = 0
                self.step = 'end'

        elif self.step == 'end':
            self.msg.angular.z = 10
            rospy.loginfo_once(f'Target Reached\nfinal position: x {self.x}, y {self.y}')
        self.publisher.publish(self.msg)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_pose()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.reach_goal()
            self.log_position()
            time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('asg_03_navigation', log_level=rospy.DEBUG)
    asg_03_navigation = Asg03Navigation()
    asg_03_navigation.run()
