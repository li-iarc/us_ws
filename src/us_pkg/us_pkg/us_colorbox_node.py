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
        self.small_rect_size = 15  # 小矩形的邊長

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

        # 定義四個角落的小矩形位置
        small_rect_positions = [
            (10, self.rect_height - self.small_rect_size - 10),  # 左上角
            (10, 10),                                            # 左下角
            (self.rect_width - self.small_rect_size - 10, self.rect_height - self.small_rect_size - 10),  # 右上角
            (self.rect_width - self.small_rect_size - 10, 10)    # 右下角
        ]

        # 定義感測器標籤的位置 # 左右移動的話調整x座標,右方為正
        text_positions = [
            (10, self.rect_height + 5),   # S1 向右移動 10 單位
            (10, -10),                    # S2 向右移動 10 單位
            (self.rect_width - 10, self.rect_height + 5),  # S3 向左移動 10 單位
            (self.rect_width - 10, -10)   # S4 向左移動 10 單位
        ]

        # 繪製小矩形並顯示感測器值和顏色
        for i, ((x, y), (tx, ty)) in enumerate(zip(small_rect_positions, text_positions)):
            distance = self.sensor_values[i]
            color = self.get_color(distance)
            
            # 繪製小矩形
            self.ax.add_patch(plt.Rectangle((x, y), self.small_rect_size, self.small_rect_size, color=color))

            # 顯示感測器值（保持原位置）
            self.ax.text(tx, ty, f"S{i + 1}: {distance:.1f} mm", ha='center', va='center', fontsize=15)

        # 設定坐標範圍
        self.ax.set_xlim(-20, self.rect_width + 20)
        self.ax.set_ylim(-20, self.rect_height + 20)
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
