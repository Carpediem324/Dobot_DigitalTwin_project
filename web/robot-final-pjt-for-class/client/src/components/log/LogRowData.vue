<script setup>
import { ref, computed } from "vue";

import { useSocketStore } from "@/stores/socket";

const socketStore = useSocketStore();

const userInputMessage = ref("");

const statusColor = computed(() => {
  if (socketStore.connectStatus === "pending") {
    return "white";
  } else if (socketStore.connectStatus === "connected") {
    return "green";
  } else {
    return "red";
  }
});

function sendMessage() {
  if (userInputMessage.value === "") return;
  try {
    // JSON 포맷인지 체크
    JSON.parse(userInputMessage.value);
    // JSON 포맷이 맞다면 메시지 전송
    socketStore.sendMessage(userInputMessage.value);
    userInputMessage.value = "";
  } catch (error) {
    alert("Invalid JSON format.");
    return;
  }
}
</script>

<template>
  <div class="logbox">
    <h2>Socket Raw Data Display</h2>
    <div class="test-input-container">
      <input
        type="text"
        v-model="userInputMessage"
        placeholder="Send message to server"
        @keyup.enter="sendMessage"
      />
      <button @click="sendMessage">전송</button>
    </div>
    <p class="connect-info">
      <span class="label">Status: </span>
      <span class="message" :style="{ color: statusColor }">{{
        socketStore.connectStatus
      }}</span>
    </p>
    <ul>
      <li v-for="(message, idx) in socketStore.messages" :key="idx">
        <p class="command">
          <span class="label">No. {{ idx + 1 }} - </span>
          <span class="content">{{ message }}</span>
        </p>
      </li>
    </ul>
  </div>
</template>

<style scoped>
.logbox {
  max-height: 525px;
  min-height: 525px;
  padding: 20px;
  margin-left: 20px;
  margin-right: 20px;
  margin-top: 60px;
  margin-bottom: 20px;
  overflow: auto;
  background-color: black;
  border-radius: 10px;
}

.logbox > h2 {
  color: white;
}

.logbox > .connect-info {
  color: white;
}

.logbox > ul > li > .cdate {
  font-weight: bold;
  color: greenyellow;
}

.logbox > .test-input-container {
  width: 300px;
}

.logbox > .test-input-container > button {
  margin-left: 10px;
}

.logbox > ul > li > .command > .label {
  font-weight: bold;
  color: green;
}

.logbox > ul > li > .command > .content {
  color: #a5d6a7;
}
</style>
