#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import time
import random

# --- 请在这里修改为你Jetson Nano的IP地址 ---
JETSON_NANO_IP = '192.168.68.52'  # 请确保IP地址正确！
# -----------------------------------------

# 全局变量，持有roslibpy客户端
client = None

def find_item_handler(request, response):
    """
    服务处理函数，接收英文标识符，返回英文标识符
    """
    # 接收到的item是'fruits', 'vegetables'或'desserts'
    item = request['item_to_find']
    print(f"[电脑端] 收到请求: 寻找 '{item}'") # f-string在Python 3.8下工作良好
    
    print(f"[电脑端] 仿真进行中...")
    time.sleep(5)
    
    # 【修改点1】定义要返回的房间标识符
    possible_rooms = ['room_A', 'room_B', 'room_C']
    found_in_room = random.choice(possible_rooms)
    
    print(f"[电脑端] 仿真结束! '{item}' 在 '{found_in_room}' 被找到。")
    
    # 【修改点2】填充response，返回纯英文标识符
    response['room_location'] = found_in_room
    
    return True

def main():
    global client
    
    try:
        client = roslibpy.Ros(host=JETSON_NANO_IP, port=9090)
        
        client.run()
        while not client.is_connected:
            print("[电脑端] 正在连接到Jetson Nano的rosbridge...")
            time.sleep(1)
            
        print('[电脑端] 连接成功！准备提供仿真服务...')

        # 确保这里的包名'ucar2pc'和您电脑上的包名一致！
        service = roslibpy.Service(client, '/find_item_simulation', 'ucar2pc/FindItem') 
        service.advertise(find_item_handler)

        print('[电脑端] 服务已启动，正在等待小车的请求...')
        
        while client.is_connected:
            time.sleep(1)

    except Exception as e:
        print(f"[电脑端] 出现错误: {e}")
    finally:
        if client is not None and client.is_connected:
            client.terminate()
        print('[电脑端] 程序已退出。')

if __name__ == '__main__':
    main()