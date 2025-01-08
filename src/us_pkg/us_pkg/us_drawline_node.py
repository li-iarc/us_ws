'''
四個感測器在特定數值範圍時改變顏色
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class SensorDataVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_data_visualizer')

        # 訂閱 /sensor_data topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.listener_callback,
            10)

        self.fig, self.ax = plt.subplots()
        self.rect_width = 110
        self.rect_height = 60

        # 初始化感測器值
        self.sensor_values = [0.0, 0.0, 0.0, 0.0]

        # 啟動 matplotlib 的互動模式
        plt.ion()
        self.plot_data()

    def listener_callback(self, msg):
        # 更新感測器值
        self.sensor_values = msg.data
        self.get_logger().info(f"Received sensor data: {self.sensor_values}")

        # 刷新圖形
        self.plot_data()

    def plot_data(self):
        self.ax.clear()

        # 繪製矩形
        self.ax.add_patch(plt.Rectangle((0, 0), self.rect_width, self.rect_height, fill=False))

        # 顯示感測器值在四個角落，並根據距離變更顏色
        for i, (x, y, label) in enumerate([(0, self.rect_height, 'S1'),
                                            (0, 0, 'S2'),
                                            (self.rect_width, self.rect_height, 'S3'),
                                            (self.rect_width, 0, 'S4')]):
            distance = self.sensor_values[i]
            color = self.get_color(distance)
            va_option = 'bottom' if y == self.rect_height else 'top'
            self.ax.text(x, y - 2 if va_option == 'top' else y + 2,  # 調整 y 方向的位置
                         f"{label}: {distance:.1f} mm",
                         ha='left' if x == 0 else 'right', va=va_option,
                         color=color, fontsize=15)

        # 設定坐標範圍
        self.ax.set_xlim(-10, self.rect_width + 10)
        self.ax.set_ylim(-10, self.rect_height + 10)
        self.ax.set_aspect('equal')

        # 顯示圖形
        plt.draw()
        plt.pause(0.01)

    def get_color(self, distance):
        """
        根據距離返回對應顏色：
        - 紅色：距離 < 50 或 > 200
        - 橘色：50 <= 距離 < 80 或 150 < 距離 <= 200
        - 綠色：80 <= 距離 <= 150
        """
        if distance < 50 or distance > 200:
            return 'red'
        elif 50 <= distance < 80 or 150 < distance <= 200:
            return 'orange'
        else:
            return 'green'

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
