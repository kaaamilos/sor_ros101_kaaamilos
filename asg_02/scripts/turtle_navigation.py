import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

TURTLESIM_ZERO = 5.544444561004639
THRESHOLD = 0.05


def identify_quarter(x, y):
    if x > TURTLESIM_ZERO and y > TURTLESIM_ZERO:
        quarter = 1
    elif x < TURTLESIM_ZERO < y:
        quarter = 2
    elif x < TURTLESIM_ZERO and y < TURTLESIM_ZERO:
        quarter = 3
    elif x > TURTLESIM_ZERO > y:
        quarter = 4
    else:
        raise ValueError('data is not correct')
    return quarter


class TurtleNavigation():
    def __init__(self, x, y):
        self.target_x = x
        self.target_y = y
        self.quarter = identify_quarter(x, y)
        self.step = 'init'
        self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.theta = 0
        self.x = TURTLESIM_ZERO
        self.y = TURTLESIM_ZERO
        self.subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.msg = Twist()
        self.speed = 0.5
        self.last_log = 0

    def pose_callback(self, data):
        self.theta = data.theta
        self.x = data.x
        self.y = data.y

    def log_position(self):
        now = time.time()
        if now > self.last_log + 1 and self.step != 'end':
            rospy.logdebug(f'current position x: {self.x}, y: {self.y}, theta: {self.theta}')
            self.last_log = time.time()

    def run(self):
        while not rospy.is_shutdown():
            if self.step == 'init':
                if self.quarter == 1 or self.quarter == 4:
                    self.step = 'reach_x'
                elif (self.quarter == 2 or self.quarter == 3) and self.theta < math.pi - THRESHOLD:
                    self.msg.angular.z = self.speed
                else:
                    self.msg.angular.z = 0
                    self.step = 'reach_x'

            elif self.step == 'reach_x':
                if self.quarter == 1 or self.quarter == 4:
                    if self.x < self.target_x:
                        self.msg.linear.x = self.speed
                    else:
                        self.msg.linear.x = 0
                        self.step = 'turn_to_reach_y'
                else:
                    if self.x > self.target_x:
                        self.msg.linear.x = self.speed
                    else:
                        self.msg.linear.x = 0
                        self.step = 'turn_to_reach_y'

            elif self.step == 'turn_to_reach_y':
                if self.quarter == 1 and self.theta < math.pi/2 - THRESHOLD:
                    self.msg.angular.z = self.speed
                elif self.quarter == 2 and self.theta > math.pi/2 - THRESHOLD:
                    self.msg.angular.z = -1 * self.speed
                elif self.quarter == 3 and (self.theta > 0 or self.theta < -1 * math.pi/2):
                    self.msg.angular.z = self.speed
                elif self.quarter == 4 and self.theta > -1 * math.pi/2:
                    self.msg.angular.z = -1 * self.speed
                else:
                    self.msg.angular.z = 0
                    self.step = 'reach_y'

            elif self.step == 'reach_y':
                if self.quarter == 1 or self.quarter == 2:
                    if self.y < self.target_y:
                        self.msg.linear.x = self.speed
                    else:
                        self.msg.linear.x = 0
                        self.step = 'end'
                else:
                    if self.y > self.target_y:
                        self.msg.linear.x = self.speed
                    else:
                        self.msg.linear.x = 0
                        self.step = 'end'
            elif self.step == 'end':
                self.msg.angular.z = 10 * self.speed
                rospy.loginfo_once(f'Target Reached, final position: x {self.x}, y {self.y}')

            self.publisher.publish(self.msg)
            self.log_position()
            time.sleep(0.01)


def main():
    rospy.init_node('turtle_navigation', log_level=rospy.DEBUG)

    x = float(input("Enter x: "))
    y = float(input("Enter y: "))

    rospy.loginfo(f'moving to x: {x}, y: {y}')

    turtle_navigation = TurtleNavigation(x, y)
    turtle_navigation.run()


if __name__ == '__main__':
    main()
