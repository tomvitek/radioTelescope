from django.apps import AppConfig


class TestappConfig(AppConfig):
    name = 'testapp'
    def ready(self):
        print("HI from testapp")
