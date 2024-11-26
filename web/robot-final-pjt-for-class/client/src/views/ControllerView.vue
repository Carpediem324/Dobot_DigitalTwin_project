<script setup>
import ControllerConveyer from "@/components/controller/ControllerConveyer.vue";
import ControllerJoint from "@/components/controller/ControllerJoint.vue";
import ControllerPrompt from "@/components/controller/ControllerPrompt.vue";

import { ref } from "vue";

import { useLogStore } from "@/stores/log";
import { useSocketStore } from "@/stores/socket";

const logStore = useLogStore();
const socketStore = useSocketStore();

const JOINT1_MIN = Number(import.meta.env.VITE_JOINT1_MIN);
const JOINT1_MAX = Number(import.meta.env.VITE_JOINT1_MAX);
const JOINT2_MIN = Number(import.meta.env.VITE_JOINT2_MIN);
const JOINT2_MAX = Number(import.meta.env.VITE_JOINT2_MAX);
const JOINT3_MIN = Number(import.meta.env.VITE_JOINT3_MIN);
const JOINT3_MAX = Number(import.meta.env.VITE_JOINT3_MAX);
const JOINT4_MIN = Number(import.meta.env.VITE_JOINT4_MIN);
const JOINT4_MAX = Number(import.meta.env.VITE_JOINT4_MAX);

const origin_conveyer_speed = ref(0);
const origin_joint1 = ref(0.0);
const origin_joint2 = ref(0.0);
const origin_joint3 = ref(0.0);
const origin_joint4 = ref(0.0);

const conveyer_speed = ref(0);
const joint1 = ref(0.0);
const joint2 = ref(0.0);
const joint3 = ref(0.0);
const joint4 = ref(0.0);

async function fetchData() {
  const data = await logStore.getLastLog();

  origin_conveyer_speed.value = Number(data.conveyer_speed);
  origin_joint1.value = Number(data.joint1);
  origin_joint2.value = Number(data.joint2);
  origin_joint3.value = Number(data.joint3);
  origin_joint4.value = Number(data.joint4);

  conveyer_speed.value = Number(data.conveyer_speed);
  joint1.value = Number(data.joint1);
  joint2.value = Number(data.joint2);
  joint3.value = Number(data.joint3);
  joint4.value = Number(data.joint4);
}

function handleConveyerSpeed(newConveyerSpeed) {
  conveyer_speed.value = newConveyerSpeed;
}

function handleJointVal(newJointVal, part) {
  if (part === "1") {
    if (newJointVal >= JOINT1_MIN && newJointVal <= JOINT1_MAX) {
      joint1.value = newJointVal;
    }
  } else if (part === "2") {
    if (newJointVal >= JOINT2_MIN && newJointVal <= JOINT2_MAX) {
      joint2.value = newJointVal;
    }
  } else if (part === "3") {
    if (newJointVal >= JOINT3_MIN && newJointVal <= JOINT3_MAX) {
      joint3.value = newJointVal;
    }
  } else if (part === "4") {
    if (newJointVal >= JOINT4_MIN && newJointVal <= JOINT4_MAX) {
      joint4.value = newJointVal;
    }
  }
}

async function createLog(newLog) {
  const result = logStore.createLog(newLog);
  return result;
}

async function power(mode) {
  try {
    socketStore.sendMessage(JSON.stringify({ program: mode }));
    await createLog({ program: mode });
  } catch (error) {
    console.error(`Power ${mode} FAILED.`);
  }
}

function reset() {
  if (confirm("처음 값으로 초기화 하시겠습니까?")) {
    conveyer_speed.value = origin_conveyer_speed.value;
    joint1.value = origin_joint1.value;
    joint2.value = origin_joint2.value;
    joint3.value = origin_joint3.value;
    joint4.value = origin_joint4.value;
    alert("초기화되었습니다. 저장하려면 SAVE 버튼을 눌러주세요.");
  }
}

async function save() {
  if (confirm("설정된 값으로 저장하시겠습니까?")) {
    socketStore.sendMessage(
      JSON.stringify({ conveyer_speed: String(conveyer_speed.value) })
    );
    socketStore.sendMessage(
      JSON.stringify({ joint1: String(joint1.value) })
    );
    socketStore.sendMessage(
      JSON.stringify({ joint2: String(joint2.value) })
    );
    socketStore.sendMessage(
      JSON.stringify({ joint3: String(joint3.value) })
    );
    socketStore.sendMessage(
      JSON.stringify({ joint4: String(joint4.value) })
    );
    // 서버의 모델이 NotNull 이기 때문에, 다른 필드까지 기입해야 함.
    const result = await createLog({
      conveyer_speed: conveyer_speed.value,
      joint1: joint1.value,
      joint2: joint2.value,
      joint3: joint3.value,
      joint4: joint4.value,
    });
    if (result) {
      alert("저장 완료!");
    } else {
      alert("실패");
    }
  }
}

fetchData();
</script>

<template>
  <div class="root root-controller">
    <div class="layout">
      <div class="conveyer-section section">
        <div class="title">
          <b>Conveyer</b>
        </div>
        <ControllerConveyer
          :conveyer_speed="conveyer_speed"
          @handleConveyerSpeed="handleConveyerSpeed"
        />
      </div>
      <div class="joint-section section">
        <div class="title">
          <b>Move Joints</b>
        </div>
        <ul class="joint-container">
          <li>
            <ControllerJoint
              part="1"
              name="Base"
              :min="JOINT1_MIN"
              :max="JOINT1_MAX"
              :val="joint1"
              @handleJointVal="handleJointVal"
            />
          </li>
          <li>
            <ControllerJoint
              part="2"
              name="Rear Arm"
              :min="JOINT2_MIN"
              :max="JOINT2_MAX"
              :val="joint2"
              @handleJointVal="handleJointVal"
            />
          </li>
          <li>
            <ControllerJoint
              part="3"
              name="Forearm"
              :min="JOINT3_MIN"
              :max="JOINT3_MAX"
              :val="joint3"
              @handleJointVal="handleJointVal"
            />
          </li>
          <li>
            <ControllerJoint
              part="4"
              name="Rotation Servo"
              :min="JOINT4_MIN"
              :max="JOINT4_MAX"
              :val="joint4"
              @handleJointVal="handleJointVal"
            />
          </li>
        </ul>
      </div>
      <div class="btns-container">
        <div class="power-btns-container">
          <button class="on-btn" @click="power('start')">START</button>
          <button class="off-btn" @click="power('stop')">STOP</button>
        </div>
        <div class="data-btns-container">
          <button class="reset-btn" @click="reset">RESET</button>
          <button class="save-btn" @click="save">SAVE</button>
        </div>
      </div>
    </div>
    <div class="layout">
      <div class="prompt-section section">
        <div class="title">
          <b>Prompt</b>
        </div>
        <ControllerPrompt />
      </div>
    </div>
  </div>
</template>

<style scoped>
.root-controller {
  display: flex;
}

.layout {
  flex: 1;
}

.section {
  padding-right: 10px;
}

.conveyer-section, .joint-section {
  min-height: 300px;
  max-height: 300px;
}

.joint-container {
  padding-top: 50px;
}

.btns-container {
  display: flex;
  justify-content: space-between;
  margin-top: 20px;
  margin-right: 10px;
}

.btns-container button {
  padding: 10px 20px;
  margin-right: 10px;
  font-size: 16px;
  border: 2px solid #000;
  border-radius: 5px;
  cursor: pointer;
}

.on-btn {
  background-color: green;
  color: white;
}

.on-btn:hover {
  background-color: rgb(0, 209, 0);
  color: white;
}

.off-btn {
  background-color: rgb(190, 0, 0);
  color: white;
}

.off-btn:hover {
  background-color: rgb(255, 0, 0);
  color: white;
}

.reset-btn {
  background-color: rgb(0, 0, 171);
  color: white;
}

.reset-btn:hover {
  background-color: rgb(0, 0, 255);
  color: white;
}

.save-btn:hover {
  background-color: white;
}
</style>
