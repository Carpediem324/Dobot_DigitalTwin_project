import json
from channels.generic.websocket import WebsocketConsumer
from channels.layers import get_channel_layer
from asgiref.sync import async_to_sync

channel_layer = get_channel_layer()

class RoboDKConsumer(WebsocketConsumer):
    def connect(self):
        self.accept()
        # 그룹에 가입
        async_to_sync(self.channel_layer.group_add)(
            'broadcast', self.channel_name
        )

    def disconnect(self, close_code):
        # 그룹에서 나가기
        self.channel_layer.group_discard('broadcast', self.channel_name)

    def receive(self, text_data):
        text_data_json = json.loads(text_data)
        message = text_data_json['message']
        response_message = message

        # 모든 연결된 클라이언트에 메시지 전송
        self.broadcast_message({
            'request': message,
            'message': response_message
        })

    def broadcast_message(self, message):
        async_to_sync(self.channel_layer.group_send)(
            'broadcast',
            {
                'type': 'send_message',
                'message': message
            }
        )

    def send_message(self, event):
        message = event['message']
        self.send(text_data=json.dumps(message))