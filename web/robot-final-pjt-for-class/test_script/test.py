import json
import time
import websocket

# dev-local
URL = "ws://localhost:8000/socket/ws/robodk/"

# production
# URL = "robots.mincodinglab.com/socket/ws/robodk/"

# JSON 명령어 리스트
commands = [
    {
        "joint1": "-90.0",
        "joint3": "80.0",
        "location_x": "0.7",
        "location_y": "0.3",
        "obj_detection_result": "red box",
        "ultrasonic_result": "20",
        "infrared_result": "false",
        "pressure_result": "512",
        "light_result": "200",
    },
    {
        "joint2": "50.0",
        "joint4": "60.0",
        "location_x": "0.1",
        "location_y": "0.2",
        "obj_detection_result": "blue box",
        "temp_result": "24",
        "ultrasonic_result": "10",
    },
    {"joint2": "30.0", "joint4": "50.0"},
    {
        "location_x": "0.2",
        "location_x": "0.3",
        "location_y": "0.5",
    },
    {
        "joint1": "-90.0",
        "joint2": "0.0",
        "joint3": "-10.0",
        "joint4": "-90.0",
        "joint1": "-90.0",
        "joint2": "0.0",
        "joint3": "-10.0",
        "joint4": "-90.0",
    },
    {
        "obj_detection_result": "green box",
        "temp_result": "23",
        "ultrasonic_result": "20",
        "infrared_result": "true",
        "pressure_result": "1024",
        "light_result": "300",
    },
    {
        "location_x": "0.7",
        "location_y": "0.2",
        "location_x": "0.1",
        "location_y": "0.1",
    },
    {
        "location_x": "0.9",
        "location_y": "0.9",
        "location_x": "0.1",
        "location_y": "0.1",
    },
    {
        "location_x": "0.1",
        "location_y": "0.1",
        "location_x": "0.9",
        "location_y": "0.9",
    },
    {
        "joint1": "90.0",
        "joint2": "85.0",
        "joint3": "95.0",
        "joint4": "90.0",
        "joint1": "90.0",
        "joint2": "85.0",
        "joint3": "95.0",
        "joint4": "90.0",
    },
    {"location_z": "0.5", "location_z": "0.5"},
    {
        "location_x": "0.5",
        "location_y": "0.5",
        "location_x": "0.4",
        "location_y": "0.4",
    },
    {"location_z": "0.3", "location_z": "0.3"},
]


def on_open(ws):
    print("Socket open!")
    while True:
        for command in commands:
            data = {"message": json.dumps(command)}
            ws.send(json.dumps(data))
            time.sleep(1)


def on_message(ws, message):
    print(f"Received: {message}")


def on_close(ws):
    print("Socket closed")


def on_error(ws, error):
    print(f"Error: {error}")


if __name__ == "__main__":
    ws = websocket.WebSocketApp(
        URL,
        on_open=on_open,
        on_message=on_message,
        on_close=on_close,
        on_error=on_error,
    )
    ws.run_forever()
