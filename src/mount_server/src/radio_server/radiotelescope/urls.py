from django.urls import path
from . import views

urlpatterns = [
    path('', views.home, name='radio_home'),
    path('scan', views.scan, name='radio_scan'),
    path('requests/status', views.request_status, name='radio_request_status'),
    path('commands/goto', views.command_goto, name='radio_cmd_goto'),
    path('commands/stop', views.command_stop, name='radio_cmd_stop'),
    path('commands/home', views.command_home, name='radio_cmd_home')
]