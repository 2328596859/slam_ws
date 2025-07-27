# -*- coding: utf-8 -*-

import os
import asyncio
import rospy
import socket
import requests
import json
from websocket_service.server import WebSocketServer

class ServerNode:
    def __init__(self):
        """初始化ROS节点和WebSocket服务器。"""
        rospy.init_node('websocket_server_node', anonymous=False)
        # socket初始化
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect(("192.168.1.200",10000))
            rospy.loginfo(f"WebSocket server listening")
        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            raise RuntimeError(f"Failed to bind socket on {host}:{port}") from e
        host = "0.0.0.0"
        port = 10780
        self.server = WebSocketServer(self.socket,host, port)
        rospy.on_shutdown(self.shutdown)
        self.loop = asyncio.new_event_loop()
        self.server_task = None
        rospy.loginfo("Kuavo WebSocket server node initialized")

    async def start_server(self):
        ws_server = await self.server.start_server()
        try:
            await asyncio.Future() 
        except asyncio.CancelledError:
            rospy.logwarn("Server task cancelled")
        finally:
            ws_server.close()
            await ws_server.wait_closed()
            rospy.loginfo("WebSocket server shut down gracefully")

    def run(self):
        """运行ROS节点和WebSocket服务器。"""
        asyncio.set_event_loop(self.loop)
        self.server_task = self.loop.create_task(self.start_server())
        try:
            self.loop.run_forever()
        except KeyboardInterrupt:
            rospy.loginfo("Keyboard interrupt received, shutting down server")
            self.shutdown()
        finally:
            try:
                self.loop.run_until_complete(self.loop.shutdown_asyncgens())
            except Exception as e:
                rospy.logwarn("Exception during loop shutdown: {}".format(e))
            self.loop.close()

    def shutdown(self):
        """关闭WebSocket服务器。"""

        if self.server_task:
            self.server_task.cancel()
            rospy.loginfo("WebSocket server task cancelled")
        # 不直接关闭loop，交由run()的finally处理
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        rospy.loginfo("Kuavo WebSocket server node shut down successfully")

def send_robotState(robotSn, status):
    """发送机器人状态到指定URL。"""
    url = "http://117.72.41.60:8080/robot/info/status/editBySn"


    payload = {
        "robotSn": robotSn,
        "status": status
    }


    headers = {
        "Content-Type": "application/json"
    }
    response = requests.put(url, data=json.dumps(payload), headers=headers)
    rospy.logwarn("状态码: %s", response.status_code)
    rospy.logwarn("响应内容: %s", response.text)


if __name__ == '__main__':
    send_robotState(robotSn="CS202504170001", status="0")
    node = ServerNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.shutdown()



# export PYTHONPATH=$PYTHONPATH:/home/shh/slam_ws/src/web_socket/src
# export PYTHONPATH=$PYTHONPATH:/home/shh/slam_ws/install_isolated/lib/python2.7/dist-packages
