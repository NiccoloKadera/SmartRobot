#!/bin/bash

# Path to log file
LOG_FILE="/home/ubuntu/rosbridge_startup.log"

echo "=== Startup Script Initiated: $(date) ===" >> "$LOG_FILE"

# 1. Wait for network to be fully up and capture the IP address
# This loop checks for an assigned IP address that isn't the loopback (127.0.0.1)
IP_ADDR=""
while [ -z "$IP_ADDR" ]; do
    IP_ADDR=$(hostname -I | awk '{print $1}')
    if [ -z "$IP_ADDR" ]; then
        echo "Waiting for IP address..." >> "$LOG_FILE"
        sleep 2
    fi
done

echo "Raspberry Pi IP Address: $IP_ADDR" >> "$LOG_FILE"

# 2. Source the ROS 2 environment
# Adjust 'humble' to your actual ROS 2 distribution name if different (e.g., jazzy, iron)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS 2 environment sourced successfully." >> "$LOG_FILE"
else
    echo "ERROR: ROS 2 setup.bash not found!" >> "$LOG_FILE"
    exit 1
fi

# 3. Launch rosbridge
echo "Launching rosbridge server..." >> "$LOG_FILE"

# Exec ensures the process replaces the shell environment, making it easier for systemd to manage
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml >> "$LOG_FILE" 2>&1