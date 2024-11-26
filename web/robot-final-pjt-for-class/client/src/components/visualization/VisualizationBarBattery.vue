<script setup>
import { ref, onMounted } from "vue";
import Chart from "chart.js/auto";

const chartRef = ref(null);

let chartInstance = null;

// 배터리 값에 따라 차트의 색상을 설정하는 함수
function getColor(value) {
  if (value <= 20) return "rgba(255, 0, 0, 0.6)"; // 20% 이하: 빨간색
  if (value <= 40) return "rgba(255, 165, 0, 0.6)"; // 40% 이하: 주황색
  if (value <= 60) return "rgba(255, 255, 0, 0.6)"; // 60% 이하: 노란색
  if (value <= 80) return "rgba(173, 255, 47, 0.6)"; // 80% 이하: 연두색
  return "rgba(0, 128, 0, 0.6)"; // 그 외: 녹색
}

function createChart() {
  const ctx = chartRef.value.getContext("2d");

  const battery = 80; // 로봇의 배터리 마지막 잔량

  // 차트 인스턴스 생성
  return new Chart(ctx, {
    type: "bar", // 차트의 타입은 막대형 차트
    data: {
      labels: ["Battery Capacity (%)"], // X축의 레이블을 설정 (현재 배터리 잔량 표시)
      datasets: [
        {
          label: "Main Battery", // 차트의 제목 (Y축 레이블)
          data: [battery], // 로봇의 배터리 데이터를 차트에 표시
          backgroundColor: [getColor(battery)], // 배터리 잔량에 따라 색상을 동적으로 설정
        },
      ],
    },
    options: {
      scales: {
        y: {
          beginAtZero: true, // Y축이 0에서 시작되도록 설정 (배터리 잔량이기 때문에 필수)
          max: 100, // Y축의 최대값을 100으로 설정 (배터리 100% 기준)
          ticks: {
            stepSize: 20, // Y축 눈금 간격을 20%로 설정
          },
        },
      },
      animation: {
        duration: 500, // 차트의 애니메이션 지속 시간 설정 (0.5초)
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
    <h2 class="card-title">Battery Copacity</h2>
    <canvas class="card-chart" ref="chartRef"></canvas>
  </div>
</template>

<style scoped>
.container {
  padding: 20px;
}
</style>
