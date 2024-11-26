<script setup>
import { ref } from "vue";
import OpenAI from "openai";
import axios from "axios";

// OpenAI 인스턴스 생성
const openai = new OpenAI({
  apiKey: import.meta.env.VITE_API_KEY, // 환경 변수에서 API 키를 가져옴
  dangerouslyAllowBrowser: true, // 프론트에서 APIKEY 노출 위험성 에러 처리
});

// 사용자 입력과 GPT 응답 상태
const userInput = ref("");
const chatLog = ref([]);

function handleKeyUp(event) {
  if (event.key === "Enter") {
    if (event.altKey) {
      // Alt + Enter: 줄 바꾸기
      userInput.value += "\n";
    } else {
      // Enter: sendQuestion 호출
      event.preventDefault();
      sendQuestion();
    }
  }
}

/**
 * @todo 대표님 지시사항으로, 시뮬레이터 동작으로 변경
 */
const requestRaspberry = async (command) => {
  console.log(command, " 요청 전송");
  try {
    const url = `${import.meta.env.VITE_RASPBERRY_URL}api/v1/robots/leds/`;
    const res = await axios.post(url, {
      command: command,
    });
    console.log(res.data);
  } catch (error) {
    console.error("Error fetching data to Raspberry Server:", error);
  }
};

const sendQuestion = async () => {
  if (userInput.value.trim() === "") return;

  // 대화 기록에 사용자 입력 추가
  chatLog.value.push({ role: "user", content: userInput.value });

  try {
    // 이전 대화 기록을 포함하여 API 요청
    const completion = await openai.chat.completions.create({
      model: "gpt-3.5-turbo",
      messages: [
        {
          role: "system",
          content: `
                상황은 나와 자유롭게 대화를 해다가, 전구를 켜야할 때 너가 답변해주면 돼.
                너는 LED 1 , LED 2, LED 3 이렇게 3개의 전구를 가지고 있어.
                각 LED 의 1, 2, 3 은 밝기야.
                만약 나의 의도가 LED 1 을 켜달라는 의도로 생각이 된다면
                only 단답형으로만 "LED 1 ON" 라고만 말해.
                그리고 꺼달라는 의도라면 only 단답형으로 "LED 1 OFF"라고 대답해
                그리고 나의 의도가 LED 2를 켜달라는 의도로 생각이 된다면
                only 단답형으로 "LED 2 ON"이라고 대답해
                그리고 꺼달라는 의도라면 only 단답형으로 "LED 2 OFF"라고 대답해
                그리고 나의 의도가 LED 3를 켜달라는 의도로 생각이 된다면
                only 단답형으로 "LED 3 ON"이라고 대답해
                그리고 꺼달라는 의도라면 only 단답형으로 "LED 3 OFF"라고 대답해
                자유롭게 대화 중에 나의 의도가 위와 같은 요청이라고 판단될 때 그때 대답하면 돼
                만약 LED 관련 의도가 아니라면, LED 관련 내용은 답변 내용에 포함하지 마
                ==================================
              `,
        },
        ...chatLog.value,
      ],
    });

    const response = completion.choices[0].message.content.trim();

    // 대화 기록에 assistant의 응답 추가
    chatLog.value.push({ role: "assistant", content: response });

    if (
      [
        "LED 1 ON",
        "LED 1 OFF",
        "LED 2 ON",
        "LED 2 OFF",
        "LED 3 ON",
        "LED 3 OFF",
      ].includes(response)
    ) {
      await requestRaspberry(response);
    }
  } catch (error) {
    console.error("Error fetching data from OpenAI API:", error);
  } finally {
    userInput.value = "";
  }
};
</script>

<template>
  <div class="chat-container">
    <div class="chat-log">
      <div v-for="(msg, index) in chatLog" :key="index" :class="msg.role">
        <span>{{ msg.content }}</span>
      </div>
    </div>
    <div class="chat-input">
      <textarea
        v-model="userInput"
        placeholder="질문을 입력하세요"
        @keyup="handleKeyUp"
      ></textarea>
      <button @click="sendQuestion">질문 보내기</button>
    </div>
  </div>
</template>

<style scoped>
.chat-container {
  min-height: 640px;
  max-height: 640px;
  margin: 10px;
}

.chat-log {
  min-height: 550px;
  max-height: 550px;
  padding-left: 2rem;
  padding-right: 2rem;
  overflow-y: auto;
  background-color: #f5f5f5;
  font-size: 1.2rem;
}

.chat-log .user {
  text-align: right;
}

.chat-log .assistant {
  text-align: left;
}

.chat-log span {
  display: inline-block;
  margin: 5px 0;
  padding: 10px;
  border-radius: 10px;
  max-width: 80%;
  word-wrap: break-word;
}

.chat-log .user span {
  background-color: #d1ecf1;
  color: #0c5460;
}

.chat-log .assistant span {
  background-color: #d4edda;
  color: #155724;
}

.chat-input {
  display: flex;
  border-top: 1px solid #c1d0d7;
  padding: 10px;
  background-color: #fff;
}

.chat-input textarea {
  flex: 1;
  height: 50px;
  padding: 10px;
  border: 1px solid #c1d0d7;
  border-radius: 5px;
  resize: none;
}

.chat-input button {
  margin-left: 10px;
  padding: 1rem;
  background-color: #007bff;
  color: #fff;
  border: none;
  border-radius: 5px;
  cursor: pointer;
  font-size: 1rem;
  font-weight: 600;
}

.chat-input button:hover {
  background-color: #0056b3;
}
</style>
