import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import matplotlib.pyplot as plt

class UltrasonicVisualizer(Node):
    def __init__(self):
        super().__init__('ultrasonic_visualizer')
        
        # 訂閱 ultrasonic_data 話題
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'ultrasonic_data',
            self.listener_callback,
            10)
        
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.sensor_values = [0] * 8  # 初始 8 個感測器數值
        
        # 設定定時器，每 0.1 秒更新一次圖形
        self.timer = self.create_timer(0.1, self.update_plot)

    def listener_callback(self, msg):
        if len(msg.data) == 8:
            self.sensor_values = msg.data
            self.get_logger().info(f'Received: {self.sensor_values}')

    def update_plot(self):
        self.ax.clear()
        
        # 畫出 110 x 60 黑色矩形
        self.ax.set_xlim(-10, 120)
        self.ax.set_ylim(-10, 70)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_title("Ultrasonic Sensor Visualization", fontsize=16)

        rect = plt.Rectangle((0, 0), 110, 60, linewidth=2, edgecolor='black', facecolor='none')
        self.ax.add_patch(rect)

        # 角落對應的新感測器索引 (按照你的新排列方式)
        corners = [(2, 58), (108, 58), (2, 15), (108, 15)]
        new_sensor_pairs = [(6, 7), (4, 5), (0, 1), (2, 3)]  # 新對應順序

        # 設定數值顏色條件
        def get_color(value):
            if value <= 0:
                return 'red'     # 負數 → 紅色
            elif value < 200:
                return 'orange'  # 小於 100 → 橘色
            else:
                return 'green'   # 其他 → 綠色

        for (x, y), (i, j) in zip(corners, new_sensor_pairs):
            value1, value2 = self.sensor_values[i], self.sensor_values[j]
            color1, color2 = get_color(value1), get_color(value2)

            # 判斷對齊方式
            if i in [0, 1, 6, 7]:  # S1, S2, S7, S8 → 置左對齊
                align = 'left'
            else:  # 其餘 (S3, S4, S5, S6) → 置右對齊
                align = 'right'

            # 顯示正確的感測器名稱和數值
            self.ax.text(x, y, f"S{i+1}: {value1}", fontsize=14, ha=align, 
                         va='top', color=color1, fontweight='bold')
            self.ax.text(x, y - 10, f"S{j+1}: {value2}", fontsize=14, ha=align, 
                         va='top', color=color2, fontweight='bold')

        plt.draw()
        plt.pause(0.01)  # 讓圖形即時更新

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicVisualizer()
    
    plt.ion()  # 開啟 Matplotlib 互動模式
    plt.show()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
