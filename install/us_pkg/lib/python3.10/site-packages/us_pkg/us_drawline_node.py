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

        # 繪製大矩形
        self.ax.add_patch(plt.Rectangle((0, 0), self.rect_width, self.rect_height, fill=False))

        # 定義四個角落的位置 (對調 sensor2 和 sensor3 的位置)
        positions = [(0, self.rect_height, 'S1'),
                    (0, 0, 'S2'),  # sensor2 和 sensor3 對調
                    (self.rect_width, self.rect_height, 'S3'),
                    (self.rect_width, 0, 'S4')]

        # 顯示感測器值並繪製小矩形
        for i, (x, y, label) in enumerate(positions):
            distance = self.sensor_values[i]
            color = self.get_color(distance)

            # 計算小矩形的左下角位置，使其完全位於大矩形內
            small_rect_x = x - self.small_rect_size / 2 if x == 0 else x - self.small_rect_size
            small_rect_y = y - self.small_rect_size / 2 if y == 0 else y - self.small_rect_size

            # 繪製小矩形
            self.ax.add_patch(plt.Rectangle((small_rect_x, small_rect_y),
                                            self.small_rect_size,
                                            self.small_rect_size,
                                            color=color))

            # 顯示感測器值
            self.ax.text(x, y - (15 if y == 0 else -5),  # 調整字的位置
                        f"{label}: {distance:.1f} mm",
                        ha='center', va='center', color=color, fontsize=15)

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
