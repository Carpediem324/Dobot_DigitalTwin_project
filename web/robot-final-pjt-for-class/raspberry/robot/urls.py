from django.urls import path
from . import views

urlpatterns = [
    path("robots/logs/", views.log_list),
    path("robots/leds/", views.control_led),
]
