import { ref } from "vue";
import { defineStore } from "pinia";

export const useSocketStore = defineStore("socketStore", () => {
  const socket = ref(null);
  const messages = ref([]);

  /**
   * 초기화 시 다음 필드를 가진다.
   * - program: String
   * - conveyer_speed: String
   * - joint1: String
   * - joint2: String
   * - joint3: String
   * - joint4: String
   * - location_x: String
   * - location_y: String
   * - location_z: String
   * - obj_detection_result: String
   * - temp_result: String
   * - ultrasonic_result: String
   * - infrared_result: String
   * - pressure_result: String
   * - light_result: String
   * - cdate: String
   */
  const currentRobotStatus = ref({});

  const connectStatus = ref("pending");
  const URL = `ws://${import.meta.env.VITE_SOCKET_URL}`;

  function connectWebSocket() {
    // WebSocket 서버 URL
    socket.value = new WebSocket(URL);

    // 연결이 열리면 호출
    socket.value.onopen = () => {
      console.log("WebSocket 연결 성공");
      connectStatus.value = "connected";
    };

    // 서버에서 메시지를 받으면 호출
    socket.value.onmessage = (event) => {
      messages.value.push(JSON.parse(event.data).message);
      const newStatus = JSON.parse(JSON.parse(event.data).message);

      // 상황에 따라 각각의 필드를 개별적으로 주기 때문에,
      // 기존 필드를 유지하기 위해 다음과 같이 처리
      if (newStatus.program !== undefined)
        currentRobotStatus.value.program = newStatus.program;
      if (newStatus.conveyer_speed !== undefined)
        currentRobotStatus.value.conveyer_speed = newStatus.conveyer_speed;
      if (newStatus.joint1 !== undefined)
        currentRobotStatus.value.joint1 = newStatus.joint1;
      if (newStatus.joint2 !== undefined)
        currentRobotStatus.value.joint2 = newStatus.joint2;
      if (newStatus.joint3 !== undefined)
        currentRobotStatus.value.joint3 = newStatus.joint3;
      if (newStatus.joint4 !== undefined)
        currentRobotStatus.value.joint4 = newStatus.joint4;
      if (newStatus.location_x !== undefined)
        currentRobotStatus.value.location_x =
          newStatus.location_x;
      if (newStatus.location_y !== undefined)
        currentRobotStatus.value.location_y =
          newStatus.location_y;
      if (newStatus.location_z !== undefined)
        currentRobotStatus.value.location_z =
          newStatus.location_z;
      if (newStatus.obj_detection_result !== undefined)
        currentRobotStatus.value.obj_detection_result =
          newStatus.obj_detection_result;
      if (newStatus.cdate !== undefined)
        currentRobotStatus.value.cdate = newStatus.cdate;
      if (newStatus.temp_result !== undefined)
        currentRobotStatus.value.temp_result = newStatus.temp_result;
      if (newStatus.ultrasonic_result !== undefined)
        currentRobotStatus.value.ultrasonic_result =
          newStatus.ultrasonic_result;
      if (newStatus.infrared_result !== undefined)
        currentRobotStatus.value.infrared_result = newStatus.infrared_result;
      if (newStatus.pressure_result !== undefined)
        currentRobotStatus.value.pressure_result = newStatus.pressure_result;
      if (newStatus.light_result !== undefined)
        currentRobotStatus.value.light_result = newStatus.light_result;
      if (newStatus.cdate !== undefined)
        currentRobotStatus.value.cdate = newStatus.cdate;
    };

    // 연결이 닫히면 호출
    socket.value.onclose = () => {
      connectStatus.value = "disconnected";
      console.log("WebSocket 연결 닫힘");
    };

    // 에러가 발생하면 호출
    socket.value.onerror = (event) => {
      connectStatus.value = "error";
      console.error("WebSocket 에러: ", event);
    };
  }

  function sendMessage(newMessage) {
    const requestObj = {
      message: newMessage,
    };
    if (socket.value && socket.value.readyState === WebSocket.OPEN) {
      socket.value.send(JSON.stringify(requestObj));
    } else {
      console.error("WebSocket 연결이 열려 있지 않습니다.");
    }
  }

  return {
    socket,
    messages,
    currentRobotStatus,
    connectStatus,
    connectWebSocket,
    sendMessage,
  };
});
