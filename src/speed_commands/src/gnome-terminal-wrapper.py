#!/usr/bin/env python3

import os

if __name__ == '__main__':
    os.system("gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun speed_commands final_with_cmdvel.py; exec bash'")
