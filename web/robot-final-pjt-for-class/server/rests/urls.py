from django.urls import path
from . import views

urlpatterns = [
    path("robots/", views.log_list),
    path("robots/<int:log_pk>", views.log_detail),
    path("robots/last/get", views.last_log_get),
]