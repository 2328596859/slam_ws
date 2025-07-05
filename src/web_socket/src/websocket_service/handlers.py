"""
Kuavo WebSocket服务的命令处理器。
"""
from websocket_service.utils import *
import rospy
import asyncio
import json
from open_msgs.msg import LiftControl, PlatformControl

class CommandHandlers:
    """
    Kuavo WebSocket服务的命令处理器。
    
    该类处理来自WebSocket客户端的命令，并执行相应的操作。
    """

    def __init__(self):
        """
        初始化命令处理器。
        """
        # 定义两个话题发布者话题/lift_control与/platform_control
        self.lift_control_publisher = rospy.Publisher('/lift_control', LiftControl, queue_size=10)
        self.platform_control_publisher = rospy.Publisher('/platform_control', PlatformControl, queue_size=10)

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
