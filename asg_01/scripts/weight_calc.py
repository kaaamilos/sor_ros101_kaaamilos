import rospy
from std_msgs.msg import Float64
from ros_tutorial.msg import Cylinder

volume = 0
volume_found = False
density = 0
density_found = False


def weight_calculate():
    pub = rospy.Publisher('/weight', Float64, queue_size=10)
    if volume_found and density_found:
        weight = volume * density
        pub.publish(weight)


def cylinder_callback(data):
    global volume
    global volume_found
    volume = data.volume
    volume_found = True


def density_callback(data):
    global density
    global density_found
    density = data.data
    density_found = True


def main():
    rospy.init_node("weight_calc")
    rospy.Subscriber('/cylinder', Cylinder, cylinder_callback)
    rospy.Subscriber('/density', Float64, density_callback)

    while not rospy.is_shutdown():
        weight_calculate()
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()


