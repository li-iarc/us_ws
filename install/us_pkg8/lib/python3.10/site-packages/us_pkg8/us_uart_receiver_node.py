# 確認串口 ls /dev/ttyUSB* /dev/ttyACM*
# 開啟對應權限sudo chmod 777 /dev/ttyUSB1

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int16MultiArray

class UltrasonicReceiver(Node):
    def __init__(self):
        super().__init__('ultrasonic_receiver')
        
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.publisher_ = self.create_publisher(Int16MultiArray, 'ultrasonic_uart_data', 10)
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.05)  # 降低 timeout，讓讀取更即時
            self.get_logger().info(f'Listening on {port} at {baudrate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            return

        self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz 讀取頻率

    def read_serial_data(self):
        raw_data = self.serial_port.read(18)  # 讀取 18 個 byte
        if len(raw_data) != 18:
            return  # 直接跳過，不輸出警告

        if raw_data[0] != 0xFF:  # 確保開頭是 FF
            return  # 直接跳過，不輸出警告

        sensor_values = self.parse_sensor_data(raw_data)
        if sensor_values:
            msg = Int16MultiArray()
            msg.data = sensor_values
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {sensor_values}')  # 只有成功收到數據才輸出

    def parse_sensor_data(self, data_bytes):
        """解析 UART 資料格式，FF 開頭，後續 16 Bytes 是 8 組感測器數據"""
        try:
            sensor_values = [int.from_bytes(data_bytes[i:i+2], byteorder='big', signed=True) 
                             for i in range(1, 17, 2)]
            return sensor_values
        except Exception:
            return None  # 避免錯誤影響程式執行

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
