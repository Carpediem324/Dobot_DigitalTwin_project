#!/bin/bash

# 1. 두봇 브링업 실행
xterm -hold -e "
source ~/magician_ros2_control_system_ws/install/setup.bash && \
ros2 launch dobot_bringup dobot_magician_control_system.launch.py
" &

# 1-1. 두봇 브링업 초기화 완료 대기
echo "Waiting for dobot_bringup to fully initialize..."
while ! ros2 node list | grep -q "/dobot_homing_srv"; do
    sleep 1
done
echo "dobot_bringup is fully initialized!"

# 2. 두봇 움직임을 RoboDK에 표현
xterm -hold -e "
source ~/magician_ros2_control_system_ws/install/setup.bash && \
ros2 run gyeong_pkg mvdkbyrobot
" &


# 3. YOLO 코드 실행
xterm -hold -e "
source ~/magician_ros2_control_system_ws/install/setup.bash && \
ros2 run gyeong_pkg sock_yolo
" &

# 3-1. 사용자 입력 대기
echo "Please check if YOLO is working properly."
echo "If everything is okay, press 'y' or 'Y' to continue."
while true; do
    read -p "Continue? (y/Y): " user_input
    if [[ "$user_input" == "y" || "$user_input" == "Y" ]]; then
        break
    else
        echo "Invalid input. Press 'y' or 'Y' to continue."
    fi
done

# 4. 두봇 무브 코드 실행
xterm -hold -e "
source ~/magician_ros2_control_system_ws/install/setup.bash && \
ros2 run gyeong_pkg dobot_mv
" &
