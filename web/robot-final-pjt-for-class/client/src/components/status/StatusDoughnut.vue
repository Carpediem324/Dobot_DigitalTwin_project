<script setup>
import { ref, onMounted, watch } from "vue";
import Chart from "chart.js/auto";

const props = defineProps({
  title: String,
  min: Number,
  max: Number,
  val: { type: [String, undefined] },
});

const chartRef = ref(null);
let chartInstance = null;

function normalizeAngle() {
  let val = 0;
  if (props.val !== undefined) val = Number(props.val);
  return ((val - props.min) / (props.max - props.min)) * 100;
}

function createDoughnutChart() {
  const ctx = chartRef.value.getContext("2d");
  const normalizedAngle = normalizeAngle();
  return new Chart(ctx, {
    type: "doughnut",
    data: {
      datasets: [
        {
          data: [normalizedAngle, 100 - normalizedAngle],
          backgroundColor: [
            "rgba(70, 130, 180, 0.7)",
            "rgba(230, 230, 230, 0.5)",
          ],
          borderColor: ["rgba(70, 130, 180, 1)", "rgba(230, 230, 230, 1)"],
          borderWidth: 1,
        },
      ],
    },
    options: {
      responsive: true,
      plugins: {
        legend: {
          display: false, // 범례를 표시하지 않음
        },
        tooltip: {
          callbacks: {
            label: (context) => {
              let val = 0;
              if (props.val !== undefined) val = Number(props.val);
              const index = context.dataIndex;
              if (index === 0) return `${String(val)}`;
              return `${String(props.max - val)}`;
            },
          },
        },
      },
      cutout: "70%", // 도넛 차트 내부를 비워줌
    },
  });
}

function updateChart() {
  const normalizedAngle = normalizeAngle();
  chartInstance.data.datasets[0].data[0] = normalizedAngle;
  chartInstance.data.datasets[0].data[1] = 100 - normalizedAngle;
  chartInstance.update();
}

onMounted(() => {
  if (chartRef.value) {
    chartInstance = createDoughnutChart();
  }
});

watch(
  () => props.val,
  () => {
    if (chartInstance && props.val !== undefined) {
      updateChart();
    }
  }
);
</script>

<template>
  <div class="doughnut-wrapper">
    <p class="banner">{{ props.title }}</p>
    <canvas ref="chartRef"></canvas>
  </div>
</template>

<style scoped>
.doughnut-wrapper {
  min-height: 200px;
  max-height: 200px;
  min-width: 200px;
  max-width: 200px;
}

.banner {
  text-align: center;
  margin: 0;
}
</style>
