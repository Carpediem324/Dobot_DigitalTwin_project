from django.shortcuts import get_list_or_404, get_object_or_404
from rest_framework.response import Response
from .serializers.response_log import LogListSerializer, LogSerializer
from rest_framework.decorators import api_view
from .models import ResponseLog

@api_view(["GET", "POST"])
def log_list(request):
    if request.method == "GET":
        logs = get_list_or_404(ResponseLog)
        serializer = LogListSerializer(logs, many=True)
        return Response(serializer.data)
    elif request.method == "POST":
        serializer = LogSerializer(data=request.data)
        if serializer.is_valid(raise_exception=True):
            serializer.save()
            return Response({"cdate":serializer.data.get("cdate")}, status=201)
        
@api_view(["GET"])
def log_detail(request, log_pk):
    log = get_object_or_404(ResponseLog, pk=log_pk)
    if request.method == "GET":
        serializer = LogSerializer(log)
        return Response(serializer.data)

@api_view(["GET"])
def last_log_get(request):
    if request.method == "GET":
        # 가장 마지막 레코드를 가져오기
        log = ResponseLog.objects.latest('id')
        serializer = LogSerializer(log)
        return Response(serializer.data)
