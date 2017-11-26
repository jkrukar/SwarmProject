#!/bin/bash
if ! grep -q "source /opt/ros/kinetic/setup.bash" ~/.bashrc
then 
  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc