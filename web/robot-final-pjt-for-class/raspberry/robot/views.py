from django.shortcuts import get_list_or_404
from rest_framework.response import Response
from .serializers.robot import LogListSerializer
from rest_framework.decorators import api_view
from .models import CommandLog
from gpiozero import LED

red_led = LED(17)
yellow_led = LED(27)
green_led = LED(22)

@api_view(["GET"])
def log_list(request):
    if request.method == "GET":
        logs = get_list_or_404(CommandLog)
        serializer = LogListSerializer(logs, many=True)
        return Response(serializer.data)

@api_view(["POST"])
def control_led(request):
    print(request)
    if request.method == "POST":
        command = request.data["command"]
        command_parse = command.split()
        if command_parse[1] == '1':
            if command_parse[2] == 'ON':
                red_led.on()
            elif command_parse[2] == 'OFF':
                red_led.off()
        elif command_parse[1] == '2':
            if command_parse[2] == 'ON':
                yellow_led.on()
            elif command_parse[2] == 'OFF':
                yellow_led.off()
        elif command_parse[1] == '3':
            if command_parse[2] == 'ON':
                green_led.on()
            elif command_parse[2] == 'OFF':
                green_led.off()

        data = {
            "command": command
        }

        serializer = LogListSerializer(data=data)
        if serializer.is_valid(raise_exception=True):
            serializer.save()
            return Response(serializer.data, status=201)
