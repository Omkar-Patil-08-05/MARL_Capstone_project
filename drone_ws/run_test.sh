#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/capstone/capstone_project_antigravity/drone_ws/install/setup.bash

export GZ_SIM_RESOURCE_PATH="/home/capstone/capstone_project_antigravity/models"
export PYTHONPATH=$PYTHONPATH:/home/capstone/capstone_project_antigravity/drone_ws/src/marl_controller/marl_controller

echo "[Test] Starting Gazebo..."
gz sim -s -r worlds/realistic_sar.sdf > /dev/null 2>&1 &
GZ_PID=$!

echo "Waiting for Gazebo to start..."
for i in {1..40}; do
  if gz topic -l | grep -q "/model/drone6/pose"; then
    echo "Gazebo ready."
    break
  fi
  sleep 1
done

sleep 5

echo "[Test] Starting Bridge..."
ros2 run ros_gz_bridge parameter_bridge /model/drone1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose /model/drone1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
/model/drone2/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose /model/drone2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
/model/drone3/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose /model/drone3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
/model/drone4/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose /model/drone4/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
/model/drone5/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose /model/drone5/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
/model/drone6/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose /model/drone6/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 10

echo "[Test] Running Test Node..."
python3 -u /home/capstone/capstone_project_antigravity/drone_ws/src/marl_controller/test/test_4m_physical_integration.py > /tmp/test_log.txt 2>&1

echo "[Test] Cleaning up..."
kill $BRIDGE_PID || true
kill $GZ_PID || true
pkill -9 -f "gz sim" || true

echo "Done"
