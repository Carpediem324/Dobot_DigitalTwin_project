<script setup>
const props = defineProps({
  title: String,
  loading: Boolean,
  logs: Array,
  parseDate: Function,
});

function parseStatus(log) {
  let str = "";
  str += "ID: " + log.id + " / ";
  str += "Program : " + log.program + " / ";
  str +=
    "Joints: " +
    log.joint1 +
    ", " +
    log.joint2 +
    ", " +
    log.joint3 +
    ", " +
    log.joint4 +
    " / ";
  str +=
    "Locations: " +
    log.location_x +
    ", " +
    log.location_y +
    ", " +
    log.location_z +
    " / ";
  str += "Object Detection: " + log.obj_detection_result + " / ";
  str += "Object Detection: " + log.obj_detection_result + " / ";
  str += "Current Temperature: " + log.temp_result + "°C" + " / ";
  str += "Obstacle Distance: " + log.ultrasonic_result + "cm" + " / ";
  str += "Infrared Detection: " + log.infrared_result + " / ";
  str += "Current Pressure: " + log.pressure_result + "Pa" + " / ";
  str += "Current Light Intensity: " + log.light_result + "lx";
  return str;
}
</script>

<template>
  <div class="logbox">
    <h2>{{ props.title }}</h2>
    <div>
      <p class="loading" v-if="props.loading">로그 데이터 수집 중...</p>
      <ul v-else>
        <li v-for="log in props.logs" :key="log.id">
          <p class="cdate">{{ props.parseDate(log.cdate) }}</p>
          <p class="command">
            <span class="label">status: </span>
            <span class="status">{{ parseStatus(log) }}</span>
          </p>
          <hr />
        </li>
      </ul>
    </div>
  </div>
</template>

<style scoped>
.logbox {
  min-height: 530px;
  max-height: 530px;
  padding: 20px;
  margin-left: 20px;
  margin-right: 20px;
  margin-top: 20px;
  margin-bottom: 20px;
  overflow: auto;
  background-color: black;
  border-radius: 10px;
}

.logbox > h2 {
  color: white;
}

.loading {
  font-weight: bold;
  color: greenyellow;
}

.cdate {
  font-weight: bold;
  color: greenyellow;
}

.command > .label {
  font-weight: bold;
  color: green;
}

.command > .status {
  color: #a5d6a7;
}
</style>
