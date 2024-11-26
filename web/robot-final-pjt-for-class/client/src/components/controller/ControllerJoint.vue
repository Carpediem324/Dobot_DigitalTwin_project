<script setup>
const props = defineProps({
  part: String,
  name: String,
  max: Number,
  min: Number,
  val: Number,
});

const emit = defineEmits(["handleJointVal"]);

let changeInterval;

function handleJointVal(evt) {
  emit("handleJointVal", Number(evt.target.value), props.part);
}

function startChangeJointVal(mode) {
  const NUM = 1;
  changeInterval = setInterval(() => {
    if (mode === "increse") {
      const newVal = Number((props.val + NUM).toFixed(2));
      emit("handleJointVal", newVal, props.part);
    } else if (mode === "decrese") {
      const newVal = Number((props.val - NUM).toFixed(2));
      emit("handleJointVal", newVal, props.part);
    }
  }, 100);
}

function stopChangeJointVal() {
  clearInterval(changeInterval);
}

function checkRange(evt) {
  if (evt.target.value > props.max) {
    evt.target.value = props.max;
    emit("handleJointVal", Number(evt.target.value), props.part);
  } else if (evt.target.value < props.min) {
    evt.target.value = props.min;
    emit("handleJointVal", Number(evt.target.value), props.part);
  }
}
</script>

<template>
  <div class="root-joint">
    <span class="label-span">{{ name }}: </span>
    <div class="range-container">
      <svg
        class="arrow l
    eft-arrow"
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        @mousedown="startChangeJointVal('decrese')"
        @mouseup="stopChangeJointVal"
        @mouseleave="stopChangeJointVal"
      >
        <polygon
          points="15,3 3,12 15,21"
          fill="#00f"
          stroke="#000"
          stroke-width="1"
        />
      </svg>
      <input
        type="range"
        :min="min"
        :max="max"
        :value="val"
        @input="handleJointVal"
        class="joint-input"
      />
      <svg
        class="arrow"
        xmlns="http://www.w3.org/2000/svg"
        viewBox="0 0 24 24"
        style="transform: rotate(180deg)"
        @mousedown="startChangeJointVal('increse')"
        @mouseup="stopChangeJointVal"
        @mouseleave="stopChangeJointVal"
      >
        <polygon
          points="15,3 3,12 15,21"
          fill="#00f"
          stroke="#000"
          stroke-width="1"
        />
      </svg>
    </div>
    <div class="input-wrapper">
      <input
        type="number"
        class="joint-text-input"
        :value="val"
        :max="max"
        :min="min"
        @input="handleJointVal"
        @change="checkRange"
      />
      <span class="unit">°</span>
    </div>
  </div>
</template>

<style scoped>
.root-joint {
  display: flex;
  align-items: center;
  margin-bottom: 12px;
}

.label-span {
  flex: 1 1 15%;
  text-align: center;
}

.range-container {
  display: flex;
  flex: 1 1 60%;
  align-items: center;
}

.arrow {
  width: 24px;
  height: 24px;
  cursor: pointer;
}

.joint-input {
  flex: 1;
  margin: 0 8px;
}

.input-wrapper {
  display: flex;
  align-items: center;
}

.input-wrapper input {
  width: 100px;
  padding: 5px;
  border: 2px solid #0056b3;
  border-radius: 5px;
  font-size: 1rem;
  color: #333;
  background-color: #fff;
  outline: none;
}

.input-wrapper input:focus {
  border-color: #007bff;
}

.input-wrapper .unit {
  margin-left: 10px;
  color: #555;
  font-size: 1rem;
}

.joint-text-input {
  margin-left: 1rem;
}
</style>
