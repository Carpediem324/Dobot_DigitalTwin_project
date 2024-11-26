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
        21, 22, 23, 24,
      ], // x축 레이블은 시간(인덱스) 값 (X-axis labels are times/indexes)
      datasets: [
        {
          label: "Temperature (°C)", // 온도 데이터 세트 (Temperature dataset)
          data: [
            25, 24, 25, 26, 27, 26, 25, 23, 21, 22, 23, 24, 22, 22, 23, 24, 25,
            26, 27, 28, 27, 26, 26, 25,
          ], // 온도 값 (Temperature values)
          backgroundColor: "rgba(255, 99, 132, 0.2)", // 배경 색상 (Background color)
          borderColor: "rgba(255, 99, 132, 1)", // 경계선 색상 (Border color)
        },
        {
          label: "Humidity (%)", // 습도 데이터 세트 (Humidity dataset)
          data: [
            60, 62, 64, 62, 60, 59, 60, 61, 57, 55, 57, 62, 60, 62, 61, 61, 57,
            56, 56, 54, 60, 62, 64, 64,
          ], // 습도 값 (Humidity values)
          backgroundColor: "rgba(54, 162, 235, 0.2)", // 배경 색상 (Background color)
          borderColor: "rgba(54, 162, 235, 1)", // 경계선 색상 (Border color)
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
    <h2 class="card-title">Temp & Humi</h2>
    <canvas class="card-chart" ref="chartRef"></canvas>
  </div>
</template>

<style scoped>
.container {
  padding: 20px;
}
</style>
