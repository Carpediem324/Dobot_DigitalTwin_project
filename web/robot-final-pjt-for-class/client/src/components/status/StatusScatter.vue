<script setup>
import { ref, onMounted, watch } from "vue";
import Chart from "chart.js/auto";

const props = defineProps({
  location_x: { type: [String, undefined] },
  location_y: { type: [String, undefined] },
  location_z: { type: [String, undefined] },
});

const chartRef = ref(null);
let chartInstance = null;

function createScatterChart() {
  let x = 0;
  let y = 0;
  let z = 0;
  if (props.location_x !== undefined) x = props.location_x;
  if (props.location_y !== undefined) y = props.location_y;
  if (props.location_z !== undefined) z = props.location_z;

  const ctx = chartRef.value.getContext("2d");
  return new Chart(ctx, {
    type: "scatter",
    data: {
      datasets: [
        {
          label: "Location",
          data: [
            {
              x: x,
              y: y,
            },
          ],
          backgroundColor: ["rgba(70, 130, 180, 0.7)"],
          pointRadius: String(Number(z) * 10), // 점의 크기는 Z
        },
      ],
    },
    options: {
      scales: {
        x: {
          type: "linear",
          min: 0, // x축 최소값 설정
          max: 1, // x축 최대값 설정 (4칸으로 제한)
          ticks: {
            stepSize: 0.1, // x축 눈금 간격을 0.1로 설정
          },
          title: {
            display: true,
            text: "X Position",
          },
        },
        y: {
          min: 0, // x축 최소값 설정
          max: 1, // x축 최대값 설정 (4칸으로 제한)
          // reverse: true, // y축을 반전시켜 좌측 상단이 (0, 0)이 되도록 함
          ticks: {
            stepSize: 0.1, // x축 눈금 간격을 1로 설정
          },
          beginAtZero: true,
          title: {
            display: true,
            text: "Y Position",
          },
        },
      },
    },
  });
}

function updateChart(x, y, z) {
  if (x !== undefined)
    chartInstance.data.datasets[0].data[0].x = props.location_x;
  if (y !== undefined)
    chartInstance.data.datasets[0].data[0].y = props.location_y;
  if (z !== undefined)
    chartInstance.data.datasets[0].pointRadius = String(
      Number(props.location_z) * 10
    );
  chartInstance.update();
}

onMounted(() => {
  if (chartRef.value) {
    chartInstance = createScatterChart();
  }
});

watch(
  () => [
    props.location_x,
    props.location_y,
    props.location_z,
  ],
  () => {
    if (chartInstance) {
      updateChart(
        props.location_x,
        props.location_y,
        props.location_z,
      );
    }
  }
);
</script>

<template>
  <div class="scatter-wrapper">
    <canvas ref="chartRef"></canvas>
  </div>
</template>
