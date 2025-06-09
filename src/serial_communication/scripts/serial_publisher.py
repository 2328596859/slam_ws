import serial


def main():
    try:
        ser = serial.Serial('/dev/pts/6', 115200, timeout=1)
        frame =[0xA5, 0x03, 0x02, 0x55, 0x20]  # 示例帧
        checksum = 0
        for b in frame:
            checksum ^= b
        frame.append(checksum)
        frame = bytes(frame)
        ser.write(frame)
        ser.flush()  # 确保数据写出
        print("Frame sent successfully.")
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
    main()