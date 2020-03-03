from django.shortcuts import render
from django.http import HttpResponse
import rospy
from mount_server.msg import position

def callback(position):
    rospy.loginfo('Position received %s', position)  

# Create your views here.
def home(request):
    print('Hi from mount_ctrl')
    rospy.Subscriber('mountPos', position, callback=callback)
    return HttpResponse("Hello django world!")