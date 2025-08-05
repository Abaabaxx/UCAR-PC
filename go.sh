#!/bin-bash

# 为每一个 roslaunch 命令打开一个新的终端窗口

# -- 选项后面的内容是新终端要执行的命令
# bash -c "..." 的作用是执行一个命令字符串
# "..." 中的 roslaunch ...; exec bash 的作用是：先执行roslaunch，当roslaunch结束后(比如你按Ctrl+C)，
# 终端不会马上关闭，而是会给你一个可交互的bash shell，方便你查看错误信息。

echo "在新终端中启动 a.launch..."
gnome-terminal -- bash -c "roslaunch package_a a.launch; exec bash"

# 等待几秒钟，确保 roscore 已经启动
sleep 5

echo "在新终端中启动 b.launch..."
gnome-terminal -- bash -c "roslaunch package_b b.launch; exec bash"

echo "所有 launch 文件已在新的终端窗口中启动。"