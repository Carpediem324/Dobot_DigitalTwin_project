import socket
from robodk import robolink  # RoboDK API
from robodk import robomath 
import time

# 연결된 RoboDK API 초기화
RDK = robolink.Robolink()

# 로봇과 관련된 Item 가져오기
RB1 = RDK.Item("Dobot Magician", itemtype=2)  # Dobot Magician 로봇

# 소켓 서버 설정
HOST = '0.0.0.0'  # 서버 IP 주소 (로봇이 실행되는 컴퓨터의 IP)
PORT = 10001      # 서버 포트 번호

# 조인트 범위 설정
JOINT_LIMITS = [
    (-90, 90),      # 1번 조인트
    (0.0, 85),      # 2번 조인트
    (-10, 95),      # 3번 조인트
    (-90, 90)       # 4번 조인트 (4번째 축 기본값 사용)
]

def main():
    # 소켓 서버 초기화
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Listening for connections on {HOST}:{PORT}...")

    connection, client_address = server_socket.accept()
    print(f"Connection established with {client_address}")

    try:
        while True:
            # RoboDK에서 현재 조인트 값 가져오기
            joint_values = RB1.Joints().list()[:4]  # 앞 3개 축 사용
            print(f"Current joint values: {joint_values}")

            # 데이터를 클라이언트로 전송
            data = ','.join(map(str, joint_values))
            connection.sendall(data.encode('utf-8'))
            time.sleep(1)  # 데이터 전송 주기
    except KeyboardInterrupt:
        print("Server interrupted by user.")
    finally:
        connection.close()
        server_socket.close()
        print("Server closed.")

if __name__ == "__main__":
    main()


