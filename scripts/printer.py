import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point, Pose


def printSpeed(data):
    #print("Got cmd_vel:" + str(data))
    print(" ")


def printTakeoff(data):
    print("Took off")


def printAtOdom():
    while not rospy.is_shutdown():
        rate = rospy.Rate(0.333)
        info = Pose()
        info.position.z = 1
        info.position.x = 1
        info.position.y = 1
        info.orientation.z = 0
        print("Published Point!")
        odometryInfo.publish(info)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("PRINTER")
    odometryInfo = rospy.Publisher('/pose', Pose, queue_size=10)
    takeoffInfo = rospy.Subscriber('/takeoff', Empty, printTakeoff)
    speedPublisher = rospy.Subscriber('/cmd_vel', Twist, printSpeed)
    printAtOdom()
