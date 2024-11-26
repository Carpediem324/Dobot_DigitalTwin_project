<script setup>
import { RouterLink } from "vue-router";
</script>

<template>
  <div class="nav">
    <RouterLink to="/controller" class="menu-btn">Controller</RouterLink>
    <RouterLink to="/visualization" class="menu-btn">Visualization</RouterLink>
    <RouterLink to="/status" class="menu-btn">Status</RouterLink>
    <RouterLink to="/log" class="menu-btn">Log</RouterLink>
  </div>
</template>

<style scoped>
.nav {
  margin-right: 1rem;
  padding: 1rem;
  border-right: 1px solid #c1d0d7;
  min-height: 660px;
  max-height: 660px;
  min-width: 100px;
  max-width: 100px;
}

.menu-btn {
  color: black;
  width: 100px;
  height: 50px;
  border: 2px solid black;
  display: flex;
  justify-content: center;
  align-items: center;
  margin-bottom: 10px;
  cursor: pointer;
  background-color: #fff;
  border-radius: 5px;
  box-shadow: 0 0 5px rgba(0, 0, 0, 0.1);
  text-decoration: none;
}

.menu-btn:hover {
  background-color: #98fb98;
}

.router-link-active {
  background-color: #87ceeb;
}
</style>
