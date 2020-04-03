from django.db import models

# Create your models here.
class MountStatus(models.Model):
    status = models.SmallIntegerField(default=0)
    hr = models.FloatField()
    dec = models.FloatField()
    scanProgress = models.SmallIntegerField(default=0)