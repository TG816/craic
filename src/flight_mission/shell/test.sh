#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: roscore
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: utils.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3;  source ~/four/first_task_ws/devel/setup.bash; roslaunch bringup location.launch' C-m

# Pane 2: simple_camera_driver.launch
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4;  source ~/four/first_task_ws/devel/setup.bash; roslaunch usb_cam simple_camera_driver.launch' C-m


# 整理第一个窗口布局
tmux select-layout -t ros_session:0 tiled

# --------------------
# 第二窗口（监控和任务）
# --------------------
tmux new-window -t ros_session:1 -n monitors_mission

# 左上：位置监控
tmux send-keys -t ros_session:1 'sleep 6; rostopic echo /mavros/local_position/pose' C-m

# 右上：视觉识别
tmux split-window -h -t ros_session:1
tmux send-keys -t ros_session:1.1 'sleep 5; source ~/four/first_task_ws/devel/setup.bash; roslaunch cloud_recognition all_noflag.launch' C-m

# 向下切分
tmux split-window -v -t ros_session:1

# ======================================
# 左下：servo 舵机启动
# ======================================
tmux send-keys -t ros_session:1.2 'sleep 10; roslaunch tutorial_catapult catapult_driver.launch' C-m

# 把左下窗格 向右切分 → 出来右下
tmux split-window -h -t ros_session:1.2

# ======================================
# 右下：flight_mission 飞行任务
# ======================================
tmux send-keys -t ros_session:1.3 'sleep 15; source ~/four/first_task_ws/devel/setup.bash; roslaunch flight_mission test.launch' C-m

# 布局对齐
tmux select-layout -t ros_session:1 tiled

# --------------------
# 进入界面
# --------------------
tmux select-window -t ros_session:0
tmux attach-session -t ros_session:1
