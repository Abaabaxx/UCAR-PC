# 使用以下指令发送目标点进行测试
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped -f /home/abaaba/ucar_ws/src/ucar_rviz/goal/ --once

# yaw 四元数
90 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}

-90 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}

180 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}

-180 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

0 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

# 30，60，90
30 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.2588, w: 0.9659}

60 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.5, w: 0.866}

120 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.866, w: 0.5}

150 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.9659, w: 0.2588}

# 将30，150度换为0与180度的参数也可以用，效果也不错

# 45 135度
45 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.3827, w: 0.9239}
  
135 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.9239, w: 0.3827}

# 40 140度
40 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.3420, w: 0.9397}
  
140 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.9397, w: 0.3420}


# 35 155度
35 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.3007, w: 0.9537}
  
155 度 Yaw：
  orientation: {x: 0.0, y: 0.0, z: 0.9747, w: 0.2239}

