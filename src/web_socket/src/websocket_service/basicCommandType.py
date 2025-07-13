"""
 WebSocket服务中使用的常量和枚举。
 便于统一管理和使用WebSocket通信的命令类型。
"""

from enum import Enum

class Command(str, Enum):
    """WebSocket通信的命令类型。"""
    LIFT_CONTROL = "LIFT_CONTROL"
    PLATFORM_CONTROL = "PLATFORM_CONTROL"
    PING = "PING"
    ERROR = "ERROR"
    WARN = "WARN"
    SUBSCRIBE = "SUBSCRIBE"
    UNSUBSCRIBE = "UNSUBSCRIBE"
    CANCLE_FLAME_ALARM = "CANCLE_FLAME_ALARM"
    CANCLE_SMOKE_ALARM = "CANCLE_SMOKE_ALARM"
    GET_FLAME_ALARM_STATE = "GET_FLAME_ALARM_STATE"
    GET_SMOKE_ALARM_STATE = "GET_SMOKE_ALARM_STATE"


