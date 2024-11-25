#!/bin/bash


# 1-1. 두봇 브링업 초기화 완료 대기
echo "Waiting for dobot_bringup to fully initialize..."
while ! ros2 node list | grep -q "/dobot_PTP_server"; do
    sleep 1
done
echo "dobot_bringup is fully initialized!"

# 2. RoboDK로 두봇 조종
xterm -hold -e "
source ~/magician_ros2_control_system_ws/install/setup.bash && \
ros2 run gyeong_pkg mvrobotbydk
" &
