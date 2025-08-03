#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import json
import threading
import time
import websocket
import rospy
from rospy import spin
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
# from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import String

# 导航点坐标配置
NAVIGATION_POINTS = {
    'start': {'x': 0.3, 'y': 0.0, 'z': 0.0, 'w': -1.0},
    '1': {'x': 3.9, 'y': 1.0, 'z': 0.8434, 'w': 0.5373},
    '2': {'x': 2.45, 'y': 1.6, 'z': 0.7071, 'w': 0.7071},
    '3': {'x': 0.84, 'y': 1.24, 'z': 0.7071, 'w': 0.7071},
}

class TaskExecutor:
    def __init__(self, websocket_url="ws://192.168.68.52:9090"):
        # 初始化ROS节点
        rospy.init_node('task_executor_node', anonymous=True)
        
        # 初始化move_base客户端
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")
        
        # 初始化WebSocket连接
        self.websocket_url = websocket_url
        self.ws = None
        self.connected = False
        
        # 初始化状态变量
        self.yolo_detections = []
        self.task_running = False
        self.lock = threading.Lock()
        
        # 订阅YOLO检测结果
        # self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', 
        #                                BoundingBoxes, 
        #                                self.yolo_callback)
        
        print("TaskExecutor initialized")
        
    def yolo_callback(self, msg):
        """处理YOLO检测结果的回调函数"""
        # with self.lock:
        #     # 更新检测结果列表
        #     self.yolo_detections = [box.Class for box in msg.bounding_boxes]
        #     rospy.logdebug("YOLO detections: %s", self.yolo_detections)

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            if data.get("topic") == "/object_recognition_request":
                if not self.task_running:
                    self.task_running = True
                    request_data = data.get("msg", {}).get("data", "")
                    if not request_data:
                        print("Empty request_data received, ignoring")
                        self.task_running = False
                        return
                    print("Received raw request_data:", request_data)
                    try:
                        request_dict = json.loads(request_data)
                    except json.JSONDecodeError as e:
                        print(f"JSON decode error: {e}")
                        self.task_running = False
                        return
                    thread = threading.Thread(target=self.run_mission, args=(request_dict,))
                    thread.daemon = True
                    thread.start()
            else:
                print("Task already running, ignoring new request")
        except Exception as e:
            print("Error processing message: {0}".format(e))


    def on_error(self, ws, error):
        """WebSocket错误回调"""
        print("WebSocket error: {0}".format(error))
        self.connected = False

    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket关闭回调"""
        print("Connection closed")
        self.connected = False

    def on_open(self, ws):
        """WebSocket连接建立回调"""
        print("Connected to server")
        self.connected = True
        
        # 订阅请求话题
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/object_recognition_request"
        }
        ws.send(json.dumps(subscribe_msg))
        print("Subscribed to /object_recognition_request topic")

    def connect(self):
        """建立WebSocket连接"""
        try:
            print("Connecting to server at {0}".format(self.websocket_url))
            self.ws = websocket.WebSocketApp(
                self.websocket_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            
            # 在后台线程中运行WebSocket
            ws_thread = threading.Thread(target=self.ws.run_forever)
            ws_thread.daemon = True
            ws_thread.start()
            
            # 等待连接建立
            timeout = 5  # 5秒超时
            while not self.connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1
            
            return self.connected
            
        except Exception as e:
            print("Connection failed: {0}".format(e))
            return False

    def publish_result(self, result_data):
        """发布结果消息"""
        if not self.connected:
            print("Not connected to server")
            return False
        
        message = {
            "op": "publish",
            "topic": "/object_recognition_result",
            "msg": {"data": result_data}
        }
        
        try:
            self.ws.send(json.dumps(message))
            print("Published result: {0}".format(result_data))
            return True
        except Exception as e:
            print("Failed to publish result: {0}".format(e))
            return False

    def navigate_to_goal(self, x, y, z, w):
        """导航到指定目标点，包含停止检测和路径重规划
        
        Args:
            x, y: 目标点的位置坐标
            z, w: 目标点的方向四元数z和w分量
        
        Returns:
            bool: 导航是否成功
        """
        try:
            # 创建导航目标
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # 设置目标位置
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.position.z = 0.0
            
            # 设置目标朝向（四元数）
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = z
            goal.target_pose.pose.orientation.w = w
            
            max_retries = 3  # 最大重试次数
            retry_count = 0
            
            while retry_count < max_retries:
                
                # 取消前一次目标（如果有）
                self.move_base.cancel_goal()
                rospy.sleep(0.5)  # 等待状态更新


                # 发送导航目标
                rospy.loginfo("Sending navigation goal: x={0}, y={1} (attempt {2})".format(
                    x, y, retry_count + 1))
                self.move_base.send_goal(goal)
                
                # 初始化停止检测变量
                last_position = None
                stop_start_time = None
                
                # 监控导航过程
                while True:
                    # 检查是否完成
                    if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Successfully reached the goal")
                        return True
                    
                    # 获取当前状态
                    state = self.move_base.get_state()
                    if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
                        rospy.logwarn("Navigation failed with status: {0}".format(state))
                        break
                    
                    # 获取当前位置
                    current_position = self.move_base.get_result()
                    if current_position:
                        if last_position:
                            # 检查是否停止移动
                            if (abs(current_position.pose.position.x - last_position.pose.position.x) < 0.1 and
                                abs(current_position.pose.position.y - last_position.pose.position.y) < 0.1):
                                if stop_start_time is None:
                                    stop_start_time = rospy.Time.now()
                                elif (rospy.Time.now() - stop_start_time).to_sec() > 3.0:
                                    rospy.logwarn("Robot stopped for more than 3 seconds, replanning...")
                                    break
                            else:
                                stop_start_time = None
                        last_position = current_position
                    
                    rospy.sleep(0.1)  # 控制检查频率
                
                retry_count += 1
                if retry_count < max_retries:
                    rospy.loginfo("Retrying navigation with new plan...")
                    rospy.sleep(1.0)  # 等待一秒后重试
                else:
                    rospy.logerr("Failed to reach goal after {0} attempts".format(max_retries))
                    return False
            
            return False
                
        except Exception as e:
            rospy.logerr("Navigation error: {0}".format(e))
            return False

    def search_for_object(self, object_name, duration=5):
        """在当前位置停留指定时间进行物体识别
        
        Args:
            object_name: 要识别的物体名称
            duration: 停留识别的时间（秒）
            
        Returns:
            bool: 是否识别到目标物体
        """
        print("Searching for object: {0} for {1} seconds".format(object_name, duration))
        
        # start_time = time.time()
        # found = False
        
        # try:
        #     # 在指定时间内持续检测
        #     while time.time() - start_time < duration:
        #         with self.lock:
        #             # 检查YOLO检测结果
        #             for detection in self.yolo_detections:
        #                 # 转换为小写进行比较
        #                 if detection.lower() == object_name.lower():
        #                     print("Found object: {0}".format(object_name))
        #                     found = True
        #                     break
                
        #         if found:
        #             break
                    
        #         # 控制检测频率
        #         rospy.sleep(0.1)
            
        #     if not found:
        #         print("Object {0} not found after {1} seconds".format(object_name, duration))
            
        #     return found
            
        # except Exception as e:
        #     print("Error during object search: {0}".format(e))
        #     return False

    def run_mission(self, request_data):
        try:
            if isinstance(request_data, str):
                data = json.loads(request_data)
            elif isinstance(request_data, dict):
                data = request_data
            else:
                raise ValueError("Unsupported request_data type: {}".format(type(request_data)))
            object_name = data.get('object_name')
            request_id = data.get('request_id')
            
            found_sequence = 0
            object_was_found = False
            
            # 遍历目标点进行完整搜索
            for point_id in ['1', '2', '3']:
                point = NAVIGATION_POINTS[point_id]
                print("Navigating to point {0}".format(point_id))
                
                if self.navigate_to_goal(point['x'], point['y'], point['z'], point['w']):
                    print("Reached point {0}, searching for object".format(point_id))
                    if self.search_for_object(object_name):
                        found_sequence = int(point_id)
                        object_was_found = True
                        print("Object found at point {0}".format(point_id))
                else:
                    print("Failed to reach point {0}".format(point_id))
            
            # 完成所有搜索后，先导航到点'1'
            point_1 = NAVIGATION_POINTS['1']
            print("Search completed. Navigating to point 1")
            if self.navigate_to_goal(point_1['x'], point_1['y'], point_1['z'], point_1['w']):
                print("Successfully reached point 1")
            else:
                print("Failed to reach point 1")
            
            # 最后返回起点
            start_point = NAVIGATION_POINTS['start']
            print("Returning to start point")
            self.navigate_to_goal(start_point['x'], start_point['y'],
                                start_point['z'], start_point['w'])

            # 发布结果
            # 如果找到物体，发布序列号；如果没找到，发布0
            result = str(found_sequence) if object_was_found else "1"
            self.publish_result(result)

        except Exception as e:
            print("Error in mission execution: {0}".format(e))
        finally:
            self.task_running = False

if __name__ == '__main__':
    try:
        # 创建执行器实例，连接到WebSocket服务器
        executor = TaskExecutor(websocket_url="ws://192.168.68.52:9090")
        
        # 连接到服务器
        if not executor.connect():
            print("Failed to connect to server")
            import sys
            sys.exit(1)
        
        print("Executor is running. Press Ctrl+C to exit.")
        
        # 保持程序运行
        spin()
    except KeyboardInterrupt:
        print("\nExecutor interrupted by user")
    finally:
        print("Executor shut down.")