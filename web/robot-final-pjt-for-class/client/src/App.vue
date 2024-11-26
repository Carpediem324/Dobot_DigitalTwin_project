<script setup>
import { RouterView } from "vue-router";
import { useSocketStore } from "@/stores/socket";
import { useLogStore } from "@/stores/log";
import AppNav from "@/components/app/AppNav.vue";
import AppFooter from "@/components/app/AppFooter.vue";

const socketStore = useSocketStore();
const logStore = useLogStore();

socketStore.connectWebSocket();

logStore.getLogs();

async function getLastLog() {
  socketStore.currentRobotStatus = await logStore.getLastLog();
}

getLastLog();
</script>

<template>
  <div class="root-container">
    <div class="main-container">
      <nav class="nav">
        <AppNav />
      </nav>
      <main class="main-content">
        <RouterView />
      </main>
    </div>
    <footer class="footer">
      <AppFooter />
    </footer>
  </div>
</template>

<style scoped>
.root-container {
  background-color: #eceeeb;
  display: flex;
  flex-direction: column;
  padding: 1.5rem;
  box-shadow: 0 0 10px rgba(0, 0, 0, 0.5);
  border-radius: 10px;
  min-height: 800px;
  max-height: 800px;
  min-width: 1400px;
  max-width: 1400px;
}

.main-container {
  display: flex;
  min-height: 730px;
  max-height: 730px;
  min-width: 1400px;
  max-width: 1400px;
}
</style>
