#!/bin/bash

# ROS2 std_msgs/Float64 메시지로 /start_server 토픽에 0 초과하는 값을 10번 발행

echo "Starting to publish Float64 messages to /start_server topic..."

for i in {1..10}
do
    # 1.0부터 10.0까지 값을 발행
    value=$i.0
    echo "Publishing message $i: $value"
    ros2 topic pub --once /start_server std_msgs/msg/Float64 "data: $value"
    sleep 0.5  # 0.5초 간격으로 발행
done

echo "Finished publishing 10 messages to /start_server topic."
