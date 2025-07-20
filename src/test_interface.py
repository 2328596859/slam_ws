import socket
import threading
import time

# 打包一个消息，消息格式为 消息头(0x88) + 消息长度4个字节 + 消息体 + 消息尾(0xAA)
def pack_msg(msg):
    msg_bytes = msg.encode("utf-8")
    msg_len = len(msg_bytes)
    msg_head = bytes.fromhex("88")
    msg_tail = bytes.fromhex("AA")
    msg_send = msg_head + msg_len.to_bytes(4, "little") + msg_bytes + msg_tail
    return msg_send

# 解析消息，消息格式为 消息头(0x88) + 消息长度4个字节 + 消息体 + 消息尾(0xAA)
def parse_msg(data):
    # 确保数据长度至少为消息头、长度和尾的长度
    if len(data) < 6:
        print("Data is too short to parse.")
        return None
    # 检查头和尾
    if data[0] == 0x88 and data[-1] == 0xAA:
        # 消息长度
        msg_len = int.from_bytes(data[1:5], "little")
        # 消息体
        msg_body = data[5:-1]
        # 消息体的长度和消息长度是否一致
        if len(msg_body) == msg_len:
            # 解析消息体
            msg = msg_body.decode("utf-8")
            return msg
        else:
            print("Message length mismatch.")
    else:
        print("Fail to parse msg: " + str(data))
    return None

# 接收服务器返回的数据的线程func
def recv_thread(s):
    while True:
        data = s.recv(2048)
        # 解析数据
        msg = parse_msg(data)
        if msg:
            print("recv=>\n" + msg)

# main函数
def main():
    # 创建socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 建立连接
    print("connect to server...")
    s.connect(("192.168.1.200", 10000))
    print("connect success")
    # 启动一个线程来接收服务器返回的数据
    threading.Thread(target=recv_thread, args=(s,)).start()
    # 主线程负责发送数据到服务器
    # "CMD": "CMD_LIST_ALL_TAG_POINT",

    msg = """
    { 
    "CMD": "CMD_CHARGE", 
    "MSG_TYPE": "CLIENT_REQUEST", 
    "QUEUE_NUMBER": 33
    } 
    """
    msg_send = pack_msg(msg)
    
    send_start_time = time.time()
    # 保持主线程不退出
    while True:
        s.send(msg_send)
        print("send=>\n" + msg)
        # 等待40ms
        time.sleep(0.5)
        # 发送时间超过20秒，退出
        if time.time() - send_start_time > 0.5:
            break

if __name__ == "__main__":
    main()
