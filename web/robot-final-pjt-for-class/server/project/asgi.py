import os  # 파이썬 실행 환경에서 운영체제(맥북이라면 macOS, 리눅스라면 Ubuntu 등)에 접근하기 위한 모듈로 사용
from channels.routing import ProtocolTypeRouter, URLRouter  # Django ASGI 애플리케이션에서 HTTP와 WebSocket 요청 등 다양한 프로토콜을 처리할 수 있도록 라우팅
from django.core.asgi import get_asgi_application  # Django의 기본 HTTP 처리를 위한 ASGI 애플리케이션을 가져와서 기존의 WSGI를 사용하는 대체
from channels.auth import AuthMiddlewareStack  # WebSocket 통신에서도 인증된 사용자가 작업할 수 있도록 인증 미들웨어를 설정
from . import routing

# Django 애플리케이션의 설정을 지정 'project.settings'는 Django 프로젝트의 설정
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'project.settings')

# ASGI 애플리케이션 설정: HTTP는 일반 요청을 처리하고, WebSocket은 실시간 통신을 처리
application = ProtocolTypeRouter({
    "http": get_asgi_application(),# HTTP 요청은 Django의 기본 ASGI 애플리케이션 처리
    "websocket": AuthMiddlewareStack(  # WebSocket 요청은 인증된 사용자가 접근할 수 있도록 AuthMiddlewareStack을 사용
        URLRouter(routing.websocket_urlpatterns)
    ),
})
