import socket
from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox

# 연결된 RoboDK API 초기화
RDK = robolink.Robolink()

# 로봇과 관련된 Item 가져오기
RB1 = RDK.Item("Dobot Magician", itemtype=2)  # Dobot Magician 로봇

# 소켓 서버 설정
HOST = '192.168.26.36'  # 서버 IP 주소 (로봇이 실행되는 컴퓨터의 IP)
PORT = 10000            # 서버 포트 번호

# 조인트 범위 설정
JOINT_LIMITS = [
    (-90, 90),      # 1번 조인트
    (0.0, 85),      # 2번 조인트
    (-10, 95),      # 3번 조인트
    (-90, 90)       # 4번 조인트 (4번째 축 기본값 사용)
]

def parse_joint_values(data):
    """수신된 데이터를 파싱하여 조인트 값을 리스트로 반환"""
    try:
        return [float(x) for x in data.decode('utf-8').strip().split(',')]
    except ValueError:
        print("Invalid joint data received:", data)
        return None

def validate_joint_values(joint_values):
    """조인트 값이 허용 범위 내에 있는지 확인"""
    for i, value in enumerate(joint_values):
        joint_min, joint_max = JOINT_LIMITS[i]
        if not (joint_min <= value <= joint_max):
            print(f"Error: Joint {i+1} value {value} is out of range ({joint_min}, {joint_max}).")
            return False
    return True

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
            # 데이터 수신
            data = connection.recv(1024)
            if not data:
                break

            print(f"Received data: {data}")
            joint_values = parse_joint_values(data)
            
            if joint_values and len(joint_values) == 3:
                # 값 검증
                if validate_joint_values(joint_values):
                    # 로봇 조인트 값을 설정 (4축 중 앞 3축만 사용)
                    print(f"Moving robot to: {joint_values}")
                    RB1.MoveJ(joint_values + [0])  # 4번째 축은 기본값(0) 설정
                else:
                    print("Invalid joint values received. Ignoring this command.")
            else:
                print("Invalid joint data length. Expected 3 values.")
    
    except KeyboardInterrupt:
        print("Server interrupted by user.")
    finally:
        connection.close()
        server_socket.close()
        print("Server closed.")

if __name__ == "__main__":
    main()
