# 使用時若找不到板子,使用sudo apt purge brltty並重開機
# 確認串口 ls /dev/ttyUSB* /dev/ttyACM*
# 開啟對應權限sudo chmod 777 /dev/ttyUSB0

# 查看即時topic內容 ros2 topic echo /ultrasonic_data

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int16MultiArray

class UltrasonicReceiver(Node):
    def __init__(self):
        super().__init__('ultrasonic_receiver')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.publisher_ = self.create_publisher(Int16MultiArray, 'ultrasonic_data', 10)
        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)

        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10Hz 讀取頻率
        self.get_logger().info(f'Listening on {port} at {baudrate} baud.')

    def read_serial_data(self):
        raw_data = self.serial_port.read(20)  # 讀取 20 個 byte
        if len(raw_data) == 20:
            hex_data = raw_data.hex().upper()
            sensor_values = self.parse_sensor_data(hex_data)
            
            if sensor_values:
                msg = Int16MultiArray()
                msg.data = sensor_values
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {sensor_values}')

    def parse_sensor_data(self, hex_data):
        if not hex_data.startswith("010310"):
            return None  # 確保資料開頭正確
        
        try:
            data_bytes = bytes.fromhex(hex_data)
            sensor_values = []
            
            for i in range(3, 19, 2):  # 解析 8 個感測器數據
                value = int.from_bytes(data_bytes[i:i+2], byteorder='big', signed=True)
                sensor_values.append(value)

            return sensor_values
        except ValueError:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
