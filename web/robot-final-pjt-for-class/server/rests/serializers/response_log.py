from rest_framework import serializers
from ..models import ResponseLog

class LogListSerializer(serializers.ModelSerializer):
    class Meta:
        model = ResponseLog
        fields = "__all__"

class LogSerializer(serializers.ModelSerializer):
    class Meta:
        model = ResponseLog
        fields = "__all__"
