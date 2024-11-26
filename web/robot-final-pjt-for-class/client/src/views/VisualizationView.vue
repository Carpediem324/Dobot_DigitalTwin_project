<script setup>
import VisualizationLineEndEffectPosition from "@/components/visualization/VisualizationLineEndEffectPosition.vue";
import VisualizationBubbleLocationFrequency from "@/components/visualization/VisualizationBubbleLocationFrequency.vue";
import VisualizationBarBattery from "@/components/visualization/VisualizationBarBattery.vue";
import VisualizationLineTempHumi from "@/components/visualization/VisualizationLineTempHumi.vue";
</script>

<template>
  <div class="root visual-root">
    <div class="row">
      <div class="card">
        <VisualizationLineEndEffectPosition />
      </div>
      <div class="card">
        <VisualizationBubbleLocationFrequency />
      </div>
    </div>
    <div class="row">
      <div class="card">
        <VisualizationBarBattery />
      </div>
      <div class="card">
        <VisualizationLineTempHumi />
      </div>
    </div>
  </div>
</template>

<style scoped>
.visual-root {
  display: flex;
  flex-direction: column;
}
.row {
  flex: 1;
  display: flex;
}
.card {
  flex: 1;
  margin: 20px;
  background-color: white;
  border-radius: 10px;
  box-shadow: 0 0 10px rgba(0, 0, 0, 0.2);
}
</style>
