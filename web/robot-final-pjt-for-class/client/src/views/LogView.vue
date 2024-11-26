<script setup>
import LogCommon from "@/components/log/LogCommon.vue";
import LogRowData from "@/components/log/LogRowData.vue";
import { useLogStore } from "@/stores/log";

import { ref } from "vue";

const logStore = useLogStore();

const loading = ref(false);

function parseDate(dateString) {
  const date = new Date(dateString);
  const year = date.getFullYear().toString().substr(2); // 2자리 연도
  const month = (date.getMonth() + 1).toString().padStart(2, "0");
  const day = date.getDate().toString().padStart(2, "0");
  const hours = date.getHours().toString().padStart(2, "0");
  const minutes = date.getMinutes().toString().padStart(2, "0");
  const seconds = date.getSeconds().toString().padStart(2, "0");

  return `${year}.${month}.${day} ${hours}:${minutes}:${seconds}`;
}
</script>

<template>
  <div class="root root-log">
    <div class="section">
      <div class="title">
        <b>Log</b>
      </div>
      <div class="left-logs">
        <div class="fetch-btn-wrapper">
          <button class="fetch-btn" @click="fetchLogs">Get Logs</button>
        </div>
        <LogCommon
          title="Simulator"
          :loading="loading"
          :logs="logStore.logs"
          :parseDate="parseDate"
        />
      </div>
    </div>
    <div class="section">
      <div class="title">
        <b>Raw Data</b>
      </div>
      <LogRowData />
    </div>
  </div>
</template>

<style scoped>
.root-log {
  display: flex;
}

.root-log > .section {
  flex: 1;
}

.left-logs {
  padding: 10px;
}

.fetch-btn-wrapper {
  display: flex;
  justify-content: right;
  margin-right: 20px;
}

.fetch-btn-wrapper > .fetch-btn {
  font-size: 1.2rem;
  border-radius: 5px;
}

.fetch-btn-wrapper > .fetch-btn:hover {
  background-color: black;
  color: white;
  cursor: pointer;
}
</style>
