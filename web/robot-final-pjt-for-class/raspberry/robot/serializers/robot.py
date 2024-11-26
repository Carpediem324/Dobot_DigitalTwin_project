from rest_framework import serializers
from ..models import CommandLog


class LogListSerializer(serializers.ModelSerializer):
    class Meta:
        model = CommandLog
        fields = "__all__"
