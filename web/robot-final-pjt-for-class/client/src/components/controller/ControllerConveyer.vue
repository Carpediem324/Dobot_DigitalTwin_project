<script setup>
defineProps({
  conveyer_speed: Number,
});

const emit = defineEmits(["handleConveyerSpeed"]);

function handleConveyerSpeed(evt) {
  emit("handleConveyerSpeed", Number(evt.target.value));
}
</script>

<template>
  <div class="root-conveyer">
    <div class="layout robot-image">
      <img src="@/assets/robot_arm.png" alt="Robot" />
    </div>
    <div class="layout tool-settings">
      <label>Conveyer Speed: </label>
      <div class="input-wrapper">
        <input
          type="number"
          :value="conveyer_speed"
          @input="handleConveyerSpeed"
        />
        <span class="unit">mm/s</span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.root-conveyer {
  padding-top: 60px;
  display: flex;
}

.layout {
  flex: 1;
}

.robot-image {
  display: flex;
  justify-content: center;
}

.robot-image > img {
  height: 130px;
  width: 130px;
}

.input-wrapper input {
  width: 100px;
  padding: 5px;
  padding-right: 10px;
  margin-top: 5px;
  margin-bottom: 5px;
  border: 2px solid #0056b3;
  border-radius: 5px;
  font-size: 1rem;
  color: #333;
  background-color: #fff;
  outline: none;
  transition: border-color 0.3s;
}

.input-wrapper input:focus {
  border-color: #007bff;
}

.input-wrapper .unit {
  margin-left: 10px;
  color: #555;
  font-size: 1rem;
}
</style>
