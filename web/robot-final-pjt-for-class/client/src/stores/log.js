import { ref } from "vue";
import { defineStore } from "pinia";

import axios from "axios";

export const useLogStore = defineStore("logStore", () => {
  const URL = import.meta.env.VITE_BACKEND_URL;

  /**
   * 초기화 시 다음 필드를 가지는 객체 배열을 가진다.
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
  const logs = ref([]);

  async function getLogs() {
    try {
      const response = await axios.get(URL);
      logs.value = response.data;
    } catch (error) {
      console.error(error);
    }
  }

  async function getLastLog() {
    try {
      const response = await axios.get(`${URL}/last/get`);
      return response.data;
    } catch (error) {
      console.error(error);
    }
  }

  async function createLog(requestLog) {
    let newLog;
    try {
      const lastLog = await getLastLog();
      newLog = lastLog;

      // null 방지
      if (requestLog.program !== undefined)
        newLog.program = String(requestLog.program);
      if (requestLog.conveyer_speed !== undefined)
        newLog.conveyer_speed = String(requestLog.conveyer_speed);
      if (requestLog.joint1 !== undefined)
        newLog.joint1 = String(requestLog.joint1);
      if (requestLog.joint2 !== undefined)
        newLog.joint2 = String(requestLog.joint2);
      if (requestLog.joint3 !== undefined)
        newLog.joint3 = String(requestLog.joint3);
      if (requestLog.joint4 !== undefined)
        newLog.joint4 = String(requestLog.joint4);
      if (requestLog.location_x !== undefined)
        newLog.location_x = String(requestLog.location_x);
      if (requestLog.location_y !== undefined)
        newLog.location_y = String(requestLog.location_y);
      if (requestLog.location_z !== undefined)
        newLog.location_z = String(requestLog.location_z);
      if (requestLog.obj_detection_result !== undefined)
        newLog.obj_detection_result = String(requestLog.obj_detection_result);
      if (requestLog.temp_result_result !== undefined)
        newLog.temp_result = String(requestLog.temp_result);
      if (requestLog.ultrasonic_result !== undefined)
        newLog.ultrasonic_result = String(requestLog.ultrasonic_result);
      if (requestLog.infrared_result !== undefined)
        newLog.infrared_result = String(requestLog.infrared_result);
      if (requestLog.pressure_result !== undefined)
        newLog.pressure_result = String(requestLog.pressure_result);
      if (requestLog.light_result !== undefined)
        newLog.light_result = String(requestLog.light_result);

      await axios.post(`${URL}/`, newLog);
      // 갱신
      await getLogs();
      return true;
    } catch (error) {
      console.error(error);
    }
  }

  return { logs, getLogs, getLastLog, createLog };
});
