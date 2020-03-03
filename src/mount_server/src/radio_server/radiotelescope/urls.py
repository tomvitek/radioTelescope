from django.urls import path
from . import views
import rospy
from mount_ctrl.msg import position

# Initialize ros node
rospy.init_node("radioserver")
rospy.loginfo("THIS IS APP INIT")


urlpatterns = [
    path('', views.home, name='mount_ctrl_home')
]