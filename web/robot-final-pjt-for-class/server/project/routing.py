from django.urls import path
from sockets.consumers import RoboDKConsumer

websocket_urlpatterns = [
    path("socket/ws/robodk/", RoboDKConsumer.as_asgi()),
]
