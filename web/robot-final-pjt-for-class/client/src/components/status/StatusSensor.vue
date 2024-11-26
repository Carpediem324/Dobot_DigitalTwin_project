<script setup>
const props = defineProps({
  obj_detection_result: String,
  temp_result: String,
  ultrasonic_result: String,
  infrared_result: String,
  pressure_result: String,
  light_result: String,
});
</script>

<template>
  <div class="root-sensors">
    <p class="field">
      <span class="label">Object Detection:</span>
      <span class="val">{{ props.obj_detection_result }}</span>
    </p>
    <p class="field">
      <span class="label">Current Temperature:</span>
      <span class="val">{{ props.temp_result }}°C</span>
    </p>
    <p class="field">
      <span class="label">Obstacle Distance:</span>
      <span class="val">{{ props.ultrasonic_result }}cm</span>
    </p>
    <p class="field">
      <span class="label">Infrared Detection:</span>
      <span class="val">{{ props.infrared_result }}</span>
    </p>
    <p class="field">
      <span class="label">Current Pressure:</span>
      <span class="val">{{ props.pressure_result }}Pa</span>
    </p>
    <p class="field">
      <span class="label">Current Light Intensity:</span>
      <span class="val">{{ props.light_result }}lx</span>
    </p>
  </div>
</template>

<style scoped>
.root-sensors {
  padding: 10px;
}

.field {
  display: flex;
}

.label {
  font-weight: bold;
  flex: 1;
}

.val {
  flex: 1;
}
</style>
