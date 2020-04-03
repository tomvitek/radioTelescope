from django.apps import AppConfig
import rospy
from . import ratelBridge


class RadiotelescopeConfig(AppConfig):
    name = 'radiotelescope'
    def ready(self):
        ratelBridge.RatelBridge.getInstance() # Initialize ROS (must be on main thread)

        