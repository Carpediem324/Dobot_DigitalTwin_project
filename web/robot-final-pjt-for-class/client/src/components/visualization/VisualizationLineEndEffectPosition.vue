<script setup>
import { ref, onMounted } from "vue";
import Chart from "chart.js/auto";

const chartRef = ref(null);

let chartInstance = null;

function createChart() {
  const ctx = chartRef.value.getContext("2d");

  return new Chart(ctx, {
    type: "line",
    data: {
      labels: [
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
      ],
      datasets: [
        {
          label: "Speed (m/s)", // 데이터셋의 라벨 설정
          data: [
            15, 20, 8, 13, 16, 5, 8, 15, 5, 18, 17, 13, 20, 19, 13, 5, 9, 19, 1, 11,
          ],
          backgroundColor: "rgba(75, 192, 192, 0.2)", // 그래프의 배경색 설정
          borderColor: "rgba(75, 192, 192, 1)", // 그래프의 선 색상 설정
        },
      ],
    },
  });
}

onMounted(() => {
  chartInstance = createChart(); // 차트 생성 함수 호출
});
</script>

<template>
  <div class="container">
    <h2 class="card-title">End Effect Position</h2>
    <canvas class="card-chart" ref="chartRef"></canvas>
  </div>
</template>

<style scoped>
.container {
  padding: 20px;
}
</style>
