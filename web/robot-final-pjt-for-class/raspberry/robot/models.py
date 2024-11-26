from django.db import models


class CommandLog(models.Model):
    command = models.CharField(max_length=50)
    cdate = models.DateTimeField(auto_now_add=True)