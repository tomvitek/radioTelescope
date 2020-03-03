import rospy
from mount_ctrl.msg import position

def callback(data):
    rospy.loginfo("Message received: %s", data)

rospy.init_node('listener', anonymous=True)
rospy.Subscriber('mountPos', position, callback)
rospy.spin()