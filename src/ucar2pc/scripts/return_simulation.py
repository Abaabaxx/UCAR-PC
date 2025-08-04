#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import time
import random

# --- 请在这里修改为你Jetson Nano的IP地址 ---
JETSON_NANO_IP = '192.168.68.52'  # 示例IP，请务必修改！
# -----------------------------------------

# 全局变量，持有roslibpy客户端
client = None

def find_item_handler(request, response): # 【修改点1】增加一个 response 参数
    """
    服务处理函数：当小车调用服务时，这个函数会被执行。
    'request' 参数是请求数据。
    'response' 参数是一个空的响应对象，我们需要填充它。
    """
    item = request['item_to_find']
    print(f"[电脑端] 收到来自小车的服务请求：开始仿真，寻找 '{item}'。")
    
    print("[电脑端] 仿真进行中，请等待5秒...")
    time.sleep(5)
    
    # 随机决定物品在哪个房间
    possible_rooms = ['A房间', 'B房间', 'C房间']
    found_in_room = random.choice(possible_rooms)
    
    print(f"[电脑端] 仿真结束！'{item}' 在 '{found_in_room}' 被找到。")
    
    # 【修改点2】填充传入的 response 对象，而不是自己创建新的
    response['room_location'] = found_in_room
    
    # 【修改点3】返回 True 表示服务处理成功
    return True

def main():
    # 声明我们要使用全局变量client
    global client
    
    try:
        # 1. 创建客户端实例
        client = roslibpy.Ros(host=JETSON_NANO_IP, port=9090)
        
        # 2. 【修改点】启动连接，并用循环等待连接成功
        client.run()
        while not client.is_connected:
            print("[电脑端] 正在连接到Jetson Nano的rosbridge...")
            time.sleep(1)
            
        print('[电脑端] 连接成功！准备提供仿真服务...')

        # 3. 创建一个服务提供者 (Service Provider)
        service = roslibpy.Service(client, '/find_item_simulation', 'ucar2pc/FindItem') # 注意: 这里我根据您的路径推断包名为pc2ucar
        service.advertise(find_item_handler)

        print('[电脑端] 服务已启动，正在等待小车的请求...')
        
        # 4. 【修改点】用循环保持程序运行，以持续提供服务
        while client.is_connected:
            time.sleep(1)

    except Exception as e:
        print(f"[电脑端] 出现错误: {e}")
    finally:
        if client is not None and client.is_connected:
            client.terminate()
        print('[电脑端] 程序已退出。')

# 确保主程序入口
if __name__ == '__main__':
    main()