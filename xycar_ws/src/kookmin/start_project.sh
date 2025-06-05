#!/bin/bash

# Exit on error
set -e

echo "Starting Xycar Project..."

# Navigate to the project root directory
cd ~/xytron_404/xycar_ws/src/kookmin

# Start Unity simulation with wine in the background
echo "Starting Unity simulation..."
wine drive.exe &
UNITY_PID=$!

# Wait a moment for Unity to initialize
sleep 5

# Start ROS TCP endpoint
echo "Starting ROS TCP endpoint..."
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000 &
ENDPOINT_PID=$!

# Wait for endpoint to initialize
sleep 2

# Navigate to the driver directory
cd ~/xytron_404/xycar_ws/src/kookmin/driver

# Start all three Python nodes
echo "Starting lane_detection.py..."
python3 lane_detection.py &
LANE_DETECTION_PID=$!

echo "Starting cone_lane_detector.py..."
python3 cone_lane_detector.py &
CONE_DETECTOR_PID=$!

echo "Starting proto_track_drive.py..."
python3 track_drive.py &
TRACK_DRIVE_PID=$!

echo "All components started successfully!"
echo "Press Ctrl+C to stop all processes"

# Wait for user to press Ctrl+C
trap "echo 'Stopping all processes...'; kill $UNITY_PID $ENDPOINT_PID $LANE_DETECTION_PID $CONE_DETECTOR_PID $TRACK_DRIVE_PID; exit" INT TERM
wait 