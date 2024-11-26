<script setup>
import StatusDoughnut from "@/components/status/StatusDoughnut.vue";
import StatusSensor from "@/components/status/StatusSensor.vue";
import StatusScatter from "@/components/status/StatusScatter.vue";

import { useSocketStore } from "@/stores/socket";

const socketStore = useSocketStore();

const JOINT1_MIN = Number(import.meta.env.VITE_JOINT1_MIN);
const JOINT1_MAX = Number(import.meta.env.VITE_JOINT1_MAX);
const JOINT2_MIN = Number(import.meta.env.VITE_JOINT2_MIN);
const JOINT2_MAX = Number(import.meta.env.VITE_JOINT2_MAX);
const JOINT3_MIN = Number(import.meta.env.VITE_JOINT3_MIN);
const JOINT3_MAX = Number(import.meta.env.VITE_JOINT3_MAX);
const JOINT4_MIN = Number(import.meta.env.VITE_JOINT4_MIN);
const JOINT4_MAX = Number(import.meta.env.VITE_JOINT4_MAX);
</script>

<template>
  <div class="root status-root">
    <div class="row first-row">
      <div class="section doughnut-section">
        <div class="title">
          <b>Joints</b>
        </div>

        <ul class="doughnut-container">
          <li>
            <StatusDoughnut
              title="Base"
              :min="JOINT1_MIN"
              :max="JOINT1_MAX"
              :val="socketStore.currentRobotStatus.joint1"
            />
          </li>
          <li>
            <StatusDoughnut
              title="Rear Arm"
              :min="JOINT2_MIN"
              :max="JOINT2_MAX"
              :val="socketStore.currentRobotStatus.joint2"
            />
          </li>
          <li>
            <StatusDoughnut
              title="Forearm"
              :min="JOINT3_MIN"
              :max="JOINT3_MAX"
              :val="socketStore.currentRobotStatus.joint3"
            />
          </li>
          <li>
            <StatusDoughnut
              title="Rotation Servo"
              :min="JOINT4_MIN"
              :max="JOINT4_MAX"
              :val="socketStore.currentRobotStatus.joint4"
            />
          </li>
        </ul>
      </div>
    </div>
    <div class="row second-row">
      <div class="section cal-work-section">
        <div class="title">
          <b>Sensors</b>
        </div>
        <StatusSensor
          :obj_detection_result="
            socketStore.currentRobotStatus.obj_detection_result
          "
          :temp_result="socketStore.currentRobotStatus.temp_result"
          :ultrasonic_result="socketStore.currentRobotStatus.ultrasonic_result"
          :infrared_result="socketStore.currentRobotStatus.infrared_result"
          :pressure_result="socketStore.currentRobotStatus.pressure_result"
          :light_result="socketStore.currentRobotStatus.light_result"
        />
      </div>
      <div class="section scatter-section">
        <div class="title">
          <b>Location</b>
        </div>
        <StatusScatter
          :location_x="socketStore.currentRobotStatus.location_x"
          :location_y="socketStore.currentRobotStatus.location_y"
          :location_z="socketStore.currentRobotStatus.location_z"
        />
      </div>
    </div>
  </div>
</template>

<style scoped>
.section {
  min-height: 330px;
  max-height: 330px;
}

.doughnut-container {
  display: flex;
  min-height: 250px;
  max-height: 250px;
  min-width: 1200px;
  max-width: 1200px;
  align-items: center;
  justify-content: space-around;
  margin-bottom: 10px;
}

.status-root > .second-row {
  display: flex;
}

.status-root > .second-row > .section {
  min-width: 603px;
  max-width: 603px;
}
</style>
