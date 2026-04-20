#!/usr/bin/env bash
# Install ROS2 Humble + Gazebo Classic + Python deps needed by the sweeper sim.
set -e

echo "[sweeper] Installing ROS2 & Gazebo packages via apt..."
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-tf-transformations \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-rviz-default-plugins \
  ros-humble-vision-msgs \
  python3-colcon-common-extensions \
  python3-numpy

# Optional but recommended for the actor in the Gazebo world
sudo apt-get install -y gazebo || true

echo "[sweeper] Done."
