"""
 Kuavo webSocket服务的实用工具函数。
"""

import os
import json
from websocket_service.basicCommandType import Command


async def send_message(websocket, cmd, data):
    """
    向客户端发送消息。
    
    Args:
        websocket: WebSocket连接
        cmd: 命令类型
        data: 消息数据
    """
    message = {
        "cmd": cmd,
        "data": data
    }
    await websocket.send(json.dumps(message))

async def send_info(websocket, cmd, message):
    """
    发送WARN警告消息。
    
    Args:
        websocket: WebSocket连接
        cmd: 命令类型
        message: 警告消息
    """
    data = {
        "message": message
    }
    await send_message(websocket, cmd, data)

async def send_error(websocket, cmd,code, message):
    """
    发送ERROR错误消息。
    
    Args:
        websocket: WebSocket连接
        code: 错误代码
        message: 错误消息
    """

    data = {
        "code": code,
        "message": message
    }
    await send_message(websocket,cmd, data)


async def send_warn(websocket, data):
    """
    发送WARN警告消息。
    Args:
        websocket: WebSocket连接
        data: 警告数据
    """
    await send_message(websocket, Command.WARN, data)

def ensure_directory(directory_path):
    """
    确保目录存在，如有必要则创建它。
    
    Args:
        directory_path: 目录路径
    """
    os.makedirs(directory_path, exist_ok=True)