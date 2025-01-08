import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from datetime import datetime
import os

class SensorDataLoggerNode(Node):
    def __init__(self):
        super().__init__('sensor_data_logger_node')
        
        # 訂閱 `sensor_data` 主題
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.subscription  # 防止未使用變數警告

        # 設定目標資料夾
        self.target_directory = "src/us_pkg/logs"  # 替換為您想要的資料夾路徑
        if not os.path.exists(self.target_directory):
            os.makedirs(self.target_directory)  # 如果資料夾不存在則建立

        # 生成檔案名稱
        current_time = datetime.now().strftime('%H%M')  # 格式化時間為 HHMM
        self.file_name = os.path.join(self.target_directory, f"us_data_{current_time}.txt")
        self.get_logger().info(f"Logging sensor data to {self.file_name}")
        
        # 開啟檔案以寫入模式
        self.file = open(self.file_name, 'w')

    def listener_callback(self, msg):
        """
        回呼函式：處理接收到的感測器資料
        """
        try:
            # 將感測器資料轉換為字串
            data_str = ', '.join([f"{value:.2f}" for value in msg.data])
            
            # 將資料寫入檔案
            self.file.write(data_str + '\n')
            self.file.flush()  # 確保即時寫入檔案
            
            # 在終端顯示接收到的資料
            self.get_logger().info(f"Received and logged data: {data_str}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to log data: {e}")

    def destroy_node(self):
        """
        覆寫節點銷毀函式，確保檔案被正確關閉
        """
        self.file.close()
        self.get_logger().info(f"File {self.file_name} closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
