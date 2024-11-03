#!/bin/zsh

# Exit on error
set -e

echo "Installing specific setuptools version..."
pip3 install setuptools==58.2.0

echo "Sourcing ROS2 Humble..."
if [ -f /opt/ros/humble/setup.zsh ]; then
    source /opt/ros/humble/setup.zsh
else
    echo "Error: ROS2 Humble setup file not found!"
    exit 1
fi

echo "Changing to workspace directory..."
if [ -d ~/mpmc_ws ]; then
    cd ~/mpmc_ws
else
    echo "Error: Workspace directory not found!"
    exit 1
fi

echo "Building workspace..."
colcon build --symlink-install

echo "Sourcing workspace setup..."
if [ -f ~/mpmc_ws/install/setup.zsh ]; then
    source ~/mpmc_ws/install/setup.zsh
else
    echo "Error: Workspace setup file not found!"
    exit 1
fi

echo "Build sequence completed successfully!"
