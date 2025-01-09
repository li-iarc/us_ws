import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
from queue import Queue
from array import array

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # Initialize serial port connection
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.get_logger().info(f"Connected to {self.ser.port} at {self.ser.baudrate} baud.")
        
        # Publisher for sensor data
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_data', 10)
        
        # Timer to periodically publish data
        self.timer = self.create_timer(0.1, self.publish_data)
        
        # Queue for storing received data
        self.data_queue = Queue()
        
        # Sensor data array (4 sensors)
        self.sensor_data = array('f', [0.0, 0.0, 0.0, 0.0])
        
        # Start a background thread for reading serial data
        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
    
    def read_serial_data(self):
        """
        Background thread for reading data from the serial port.
        """
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline()
                self.data_queue.put(raw_data)
                self.ser.reset_input_buffer()  # Clear the buffer to avoid accumulation
    
    def publish_data(self):
        """
        Timer callback to publish sensor data.
        """
        while not self.data_queue.empty():
            raw_data = self.data_queue.get()
            self.parse_and_update(raw_data)
        
        # Create and publish the message
        msg = Float32MultiArray()
        msg.data = [float(value) for value in self.sensor_data]
        self.publisher_.publish(msg)
        
        # Output the sensor data as an array
        self.get_logger().info(f"Published sensor data: {list(self.sensor_data)}")

    
    def parse_and_update(self, raw_data):
        """
        Parse the raw data from the sensor and update the sensor data array.
        """
        try:
            decoded_data = raw_data.decode('utf-8').strip()
            if "sensor" in decoded_data and "Distance" in decoded_data:
                parts = decoded_data.split()
                sensor_index = int(parts[0].replace("sensor", "")) - 1
                distance = float(parts[2])
                self.sensor_data[sensor_index] = distance
        except Exception as e:
            self.get_logger().error(f"Failed to parse data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
