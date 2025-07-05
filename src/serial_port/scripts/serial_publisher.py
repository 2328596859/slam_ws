import serial

# 升降杆上升1mm
frame_Lift_up=[0xA5, 0x04, 0x55, 0x20, 0x00, 0x01, 0xD5]
# 升降杆下降1mm
frame_Lift_down=[0xA5, 0x04, 0x55, 0x20, 0x01, 0x01,0xD4 ]
# 云台左转0度
frame_Platform_left = [0xA5, 0x04, 0x55, 0x21, 0x00, 0x00, 0xD5]
# 云台右转0度
frame_Platform_right = [0xA5, 0x04, 0x55, 0x21, 0x01, 0x00, 0xD4]
# 云台上仰0度
frame_Platform_up = [0xA5, 0x04, 0x55, 0x21, 0x02, 0x00, 0xD7]
# 云台下俯0度
frame_Platform_down = [0xA5, 0x04, 0x55, 0x21, 0x03, 0x00, 0xD6]
# 烟雾温湿度请求
frame_Smoke_request = [0xA5, 0x02, 0x55, 0x22,0xD0]

def  checksum(frame):
    """计算校验和"""
    checksum = 0
    for b in frame:
        checksum ^= b
    return checksum
def publish_frame(frame):
    """发送帧到串口"""
    try:
        ser = serial.Serial('/dev/ttyS3', 115200, timeout=1)
        frame_bytes = bytes(frame)
        ser.write(frame_bytes)
        ser.flush()
        print(f"Frame sent successfully: {frame}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    publish_frame(frame_Lift_up) 
