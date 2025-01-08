import serial
import time

# 配置 UART 端口和波特率
SERIAL_PORT = "/dev/ttyACM0"  # 根據實際情況更改，例如 "/dev/ttyUSB0"
BAUD_RATE = 115200

# 初始化陣列用於儲存感測器資料
sensor_data = [None, None, None, None]  # [Sensor1, Sensor2, Sensor3, Sensor4]

def parse_sensor_data(data):
    """
    解析來自 STM32 的感測器資料，更新對應的陣列值。
    假設資料格式為: SensorX Distance: XXXX mm
    """
    global sensor_data
    try:
        decoded_data = data.decode('utf-8').strip()
        print(f"接收到的資料: {decoded_data}")  # 打印接收到的原始資料
        
        # 判斷是哪一個感測器的資料，並提取距離值
        if decoded_data.startswith("sensor1"):
            distance = int(decoded_data.split(":")[1].strip().split()[0])
            sensor_data[0] = distance
        elif decoded_data.startswith("sensor2"):
            distance = int(decoded_data.split(":")[1].strip().split()[0])
            sensor_data[1] = distance
        elif decoded_data.startswith("sensor3"):
            distance = int(decoded_data.split(":")[1].strip().split()[0])
            sensor_data[2] = distance
        elif decoded_data.startswith("sensor4"):
            distance = int(decoded_data.split(":")[1].strip().split()[0])
            sensor_data[3] = distance

        # 刷新陣列並打印
        print("更新後的陣列: ", sensor_data)

    except (ValueError, IndexError, UnicodeDecodeError) as e:
        print(f"資料解析失敗: {data}, 錯誤: {e}")

def main():
    # 初始化串列連接
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"已連接至 {SERIAL_PORT}，波特率: {BAUD_RATE}")
    except serial.SerialException as e:
        print(f"無法打開串列端口 {SERIAL_PORT}: {e}")
        return

    # 開始接收資料
    try:
        while True:
            if ser.in_waiting > 0:  # 如果串列端口有資料可讀
                raw_data = ser.readline()  # 讀取一行資料
                parse_sensor_data(raw_data)

            time.sleep(0.1)  # 避免過於頻繁地查詢串列端口
    except KeyboardInterrupt:
        print("\n終止程式。")
    finally:
        ser.close()
        print("已關閉串列端口。")

if __name__ == "__main__":
    main()
