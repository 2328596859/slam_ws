"""
kuavo WebSocket服务的WebSocket服务器实现。
"""
import asyncio
import json
import logging
import websockets
import traceback

from websocket_service.basicCommandType import Command
from websocket_service.utils import *
from websocket_service.handlers import CommandHandlers

class WebSocketServer:
    """
    kuavo WebSocket服务的WebSocket服务器。

    该类管理WebSocket连接并将命令
    路由到适当的处理器。
    """

    def __init__(self, host="localhost", port=10780):
        """
        初始化WebSocket服务器。

        Args:
            host: 要绑定的主机地址（默认：localhost）
            port: 要监听的端口（默认：10780）
        """
        self.host = host
        self.port = port
        self.handlers = CommandHandlers()
        

    async def start_server(self):
        """
        启动WebSocket服务器。

        Returns:
            websockets.server.WebSocketServer: 运行中的服务器实例
        """
        server = await websockets.serve(self.handle_connection, self.host, self.port)
        # print(f"WebSocket server started at ws://{self.host}:{self.port}")  
        print("WebSocket server started at ws://{}:{}".format(self.host, self.port))  
    
    async def handle_connection(self, websocket):
        """
        处理新的WebSocket连接。 
        接收命令并调用相应的处理器。
        Args:
            websocket: WebSocket连接对象
        """
        # print(f"New connection from {websocket.remote_address}")
        print("New connection from {}".format(websocket.remote_address))  
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    cmd = data.get("CMD")
                    await self.route_command(websocket, cmd, data)
                except json.JSONDecodeError:
                    await send_error(websocket, 400, "Invalid JSON format")
        except Exception as e:
            # print(f"连接异常: {e}")
            print("Connection error: {}".format(e))
            print(traceback.format_exc())
        finally:
            # print(f"Connection closed: {websocket.remote_address}")
            print("Connection closed: {}".format(websocket.remote_address))
      

    async def route_command(self, websocket, cmd, data):
        if cmd == Command.LIFT_CONTROL:
            await self.handlers.handle_lift_control(websocket, data)
        elif cmd == Command.PLATFORM_CONTROL:
            await self.handlers.handle_platform_control(websocket, data)
        elif cmd == Command.PING:
            await self.handlers.handle_ping(websocket, data)
        else:
            await send_error(websocket, 400, f"Unknown command: {cmd}")
 