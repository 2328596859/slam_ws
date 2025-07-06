from websocket_service.utils import *
import rospy
import asyncio
import json
from open_msgs.msg import LiftControl, PlatformControl
import std_msgs.msg
import threading

class CommandHandlers:
    """
    Kuavo WebSocket服务的命令处理器。
    
    该类处理来自WebSocket客户端的命令，并执行相应的操作。
    """

    def __init__(self):
        """
        初始化命令处理器。
        """
        # 存储WebSocket连接
        self.websocket_clients = set()
        
        # 存储事件循环引用
        self.loop = None
        
        # ROS订阅者
        self.smoke_sub = rospy.Subscriber("smokesensor/smoke_data", std_msgs.msg.Float64, self.smoke_callback)
        self.temperature_sub = rospy.Subscriber("smokesensor/temperature_data", std_msgs.msg.Float64, self.temperature_callback)
        self.humidity_sub = rospy.Subscriber("smokesensor/humidity_data", std_msgs.msg.Float64, self.humidity_callback)
        self.flame_sub = rospy.Subscriber("flamesensor/flame_data", std_msgs.msg.Int32, self.flame_callback)
        
        # 定义两个话题发布者话题/lift_control与/platform_control
        self.lift_control_publisher = rospy.Publisher('/lift_control', LiftControl, queue_size=10)
        self.platform_control_publisher = rospy.Publisher('/platform_control', PlatformControl, queue_size=10)

    def set_event_loop(self, loop):
        """设置事件循环引用"""
        self.loop = loop

    def add_websocket_client(self, websocket):
        """添加WebSocket客户端"""
        self.websocket_clients.add(websocket)

    def remove_websocket_client(self, websocket):
        """移除WebSocket客户端"""
        self.websocket_clients.discard(websocket)

    async def broadcast_to_clients(self, message):
        """向所有连接的WebSocket客户端广播消息"""
        if not self.websocket_clients:
            return
            
        # 创建任务列表
        tasks = []
        disconnected_clients = []
        
        for websocket in self.websocket_clients:
            try:
                tasks.append(websocket.send(json.dumps(message)))
            except Exception as e:
                disconnected_clients.append(websocket)
                rospy.logwarn(f"WebSocket客户端断开连接: {e}")
        
        # 移除断开的客户端
        for client in disconnected_clients:
            self.websocket_clients.discard(client)
        
        # 并发发送消息
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def _schedule_broadcast(self, message):
        """安全地调度广播任务到事件循环"""
        if self.loop and not self.loop.is_closed():
            try:
                # 使用 call_soon_threadsafe 从其他线程安全地调度协程
                future = asyncio.run_coroutine_threadsafe(
                    self.broadcast_to_clients(message), 
                    self.loop
                )
            except Exception as e:
                rospy.logwarn(f"调度广播任务失败: {e}")
        else:
            rospy.logwarn("事件循环未设置或已关闭，无法广播消息")

    def smoke_callback(self, msg):
        """烟雾传感器数据回调"""
        message = {
            "TYPE": "SENSOR_DATA",
            "SENSOR_TYPE": "SMOKE",
            "DATA": msg.data,
            "TIMESTAMP": rospy.Time.now().to_sec()
        }
        self._schedule_broadcast(message)

    def temperature_callback(self, msg):
        """温度传感器数据回调"""
        message = {
            "TYPE": "SENSOR_DATA",
            "SENSOR_TYPE": "TEMPERATURE",
            "DATA": msg.data,
            "TIMESTAMP": rospy.Time.now().to_sec()
        }
        self._schedule_broadcast(message)

    def humidity_callback(self, msg):
        """湿度传感器数据回调"""
        message = {
            "TYPE": "SENSOR_DATA",
            "SENSOR_TYPE": "HUMIDITY",
            "DATA": msg.data,
            "TIMESTAMP": rospy.Time.now().to_sec()
        }
        self._schedule_broadcast(message)

    def flame_callback(self, msg):
        """火焰传感器数据回调"""
        message = {
            "TYPE": "SENSOR_DATA",
            "SENSOR_TYPE": "FLAME",
            "DATA": msg.data,
            "TIMESTAMP": rospy.Time.now().to_sec()
        }
        self._schedule_broadcast(message)


    async def handle_subscribe(self, websocket, data):
        """处理订阅请求"""
        request_data = data.get("DATA", {})
        topics = request_data.get("TOPICS", [])
        
        # 添加WebSocket客户端到订阅列表
        self.add_websocket_client(websocket)
        
        await send_info(websocket, "SUBSCRIBE", f"已订阅话题: {topics}")

    async def handle_unsubscribe(self, websocket, data):
        """处理取消订阅请求"""
        # 从订阅列表中移除WebSocket客户端
        self.remove_websocket_client(websocket)
        
        await send_info(websocket, "UNSUBSCRIBE", "已取消订阅")

    # ...existing code...
    async def handle_lift_control(self, websocket,data):
        request_data = data.get("DATA")
        if not request_data:
            await send_error(websocket, 400, "请求数据不能为空")
            return
        direction = request_data.get("DIRECTION")
        data = request_data.get("DATA")
        if direction is None or data is None:
            await send_error(websocket, 400, "请求数据格式错误")
            return
        # 发布LiftControl消息
        lift_control_msg = LiftControl()
        if direction == "UP":
            lift_control_msg.cmd = "up"
        elif direction == "DOWN":
            lift_control_msg.cmd = "down"
        lift_control_msg.data = int(data) 
        self.lift_control_publisher.publish(lift_control_msg)
        await send_info(websocket,"LIFT_CONTROL","Lift control command sent successfully")

    async def handle_platform_control(self, websocket, data):
        request_data = data.get("DATA")
        if not request_data:
            await send_error(websocket, 400, "请求数据不能为空")
            return
        direction = request_data.get("DIRECTION")
        data = request_data.get("DATA")
        if direction is None or data is None:
            await send_error(websocket, 400, "请求数据格式错误")
            return
        # 发布PlatformControl消息
        platform_control_msg = PlatformControl()
        if direction == "UP":
            platform_control_msg.cmd = "up"
        elif direction == "DOWN":
            platform_control_msg.cmd = "down"
        elif direction == "RIGHT":
            platform_control_msg.cmd = "right"
        elif direction == "LEFT":
            platform_control_msg.cmd = "left"
        platform_control_msg.data = int(data) 
        self.platform_control_publisher.publish(platform_control_msg)
        await send_info(websocket, "PLATFORM_CONTROL", "Platform control command sent successfully")

    async def handle_ping(self, websocket, data):
        await send_info(websocket, "PING", "Ping command received successfully")