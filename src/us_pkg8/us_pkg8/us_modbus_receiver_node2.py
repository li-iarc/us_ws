### 此程式為同時使用兩個modbus轉接盒時使用,僅差在port

# 使用時若找不到板子,使用sudo apt purge brltty並重開機
# 確認串口 ls /dev/ttyUSB* /dev/ttyACM*
# 開啟對應權限sudo chmod 777 /dev/ttyUSB0

# 查看即時topic內容 ros2 topic echo /ultrasonic_data

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int16MultiArray
import struct

class UltrasonicReceiver(Node):
    def __init__(self):
        super().__init__('ultrasonic_receiver')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.publisher_ = self.create_publisher(Int16MultiArray, 'ultrasonic_data_2', 10)
        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)

        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10Hz 讀取頻率
        self.get_logger().info(f'Listening on {port} at {baudrate} baud.')

    def read_serial_data(self):
        raw_data = self.serial_port.read(21)  # 讀取 21 個 byte (含 CRC)
        if len(raw_data) == 21:
            hex_data = raw_data.hex().upper()
            self.get_logger().info(f'Received: {hex_data}')  # 印出收到的資料

            # 檢查 CRC
            if self.verify_crc(raw_data):
                sensor_values = self.parse_sensor_data(raw_data)
                if sensor_values:
                    msg = Int16MultiArray()
                    msg.data = sensor_values
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: {sensor_values}')
            else:
                self.get_logger().warn(f'CRC Error: {hex_data}')  # 印出 CRC 錯誤的資料

    def parse_sensor_data(self, raw_data):
        """ 解析 8 個超聲波感測器數據 """
        if raw_data[:3] != b'\x01\x03\x10':  # 確保開頭正確
            return None
        
        try:
            sensor_values = []
            for i in range(3, 19, 2):  # 解析 8 個感測器數據
                value = int.from_bytes(raw_data[i:i+2], byteorder='big', signed=True)
                sensor_values.append(value)
            return sensor_values
        except ValueError:
            return None

    def verify_crc(self, data):
        """ 驗證 Modbus CRC-16 """
        if len(data) < 2:
            return False
        received_crc = data[-2:]  # 最後兩個 byte 是 CRC
        calculated_crc = self.modbus_crc16(data[:-2])  # 計算 CRC
        return received_crc == calculated_crc

    def modbus_crc16(self, data):
        """ 計算 Modbus CRC-16 """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)  # 轉成小端序的 2-byte 格式

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
