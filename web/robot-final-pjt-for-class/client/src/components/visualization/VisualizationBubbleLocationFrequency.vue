<script setup>
import { ref, onMounted } from "vue";
import Chart from "chart.js/auto";

const chartRef = ref(null);

let chartInstance = null;

function getRandomData(count, max) {
  return Array.from({ length: count }, () => ({
    x: Math.random().toFixed(2), // 0.0 ~ 1.0 사이 랜덤 값
    y: Math.random().toFixed(2), // 0.0 ~ 1.0 사이 랜덤 값
    r: (Math.random() * max).toFixed(2), // 버블 크기 랜덤 값
  }));
}

function createChart() {
  const ctx = chartRef.value.getContext("2d");

  return new Chart(ctx, {
    type: "bubble",
    data: {
      datasets: [
        {
          label: "Robot A",
          backgroundColor: "rgba(255, 99, 132, 0.2)", // 트렌디한 빨간색
          borderColor: "rgba(255, 99, 132, 1)",
          data: getRandomData(10, 20), // 데이터 개수와 최대 버블 크기 설정
        },
        {
          label: "Robot B",
          backgroundColor: "rgba(54, 162, 235, 0.2)", // 트렌디한 파란색
          borderColor: "rgba(54, 162, 235, 1)",
          data: getRandomData(10, 20), // 데이터 개수와 최대 버블 크기 설정
        },
      ],
    },
    options: {
      scales: {
        x: {
          min: 0.0,
          max: 1.0,
          title: {
            display: true,
            text: "X Axis",
          },
        },
        y: {
          min: 0.0,
          max: 1.0,
          title: {
            display: true,
            text: "Y Axis",
          },
        },
      },
      plugins: {
        tooltip: {
          callbacks: {
            label: function (context) {
              const label = context.dataset.label || "";
              const x = context.raw.x;
              const y = context.raw.y;
              const r = context.raw.r;
              return `${label}: (${x}, ${y}), Frequency: ${r}`;
            },
          },
        },
      },
    },
  });
}

onMounted(() => {
  chartInstance = createChart(); // 차트 생성 함수 호출
});
</script>

<template>
  <div class="container">
    <h2 class="card-title">Location Frequency</h2>
    <canvas class="card-chart" ref="chartRef"></canvas>
  </div>
</template>

<style scoped>
.container {
  padding: 20px;
}
</style>
