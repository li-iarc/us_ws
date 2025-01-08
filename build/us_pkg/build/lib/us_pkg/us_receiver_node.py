import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from array import array

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # Initialize serial port connection
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        self.get_logger().info(f"Connected to {self.ser.port} at {self.ser.baudrate} baud.")
        
        # Publisher for sensor data
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_data', 10)
        
        # Timer to periodically read data from serial
        timer_period = 0.1  # Timer interval in seconds (100ms)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize sensor data array (4 sensors)
        self.sensor_data = array('f', [0.0, 0.0, 0.0, 0.0])
    
    def timer_callback(self):
        """
        Timer callback to publish sensor data and read from the serial port.
        """
        if self.ser and self.ser.in_waiting > 0:
            raw_data = self.ser.readline()  # Read one line of data
            self.parse_and_publish(raw_data)
    
    def parse_and_publish(self, raw_data):
        """
        Parse the raw data from the sensor and publish it.
        """
        try:
            decoded_data = raw_data.decode('utf-8').strip()
            # Parse the sensor number and distance
            if "sensor" in decoded_data and "Distance" in decoded_data:
                parts = decoded_data.split()
                sensor_index = int(parts[0].replace("sensor", "")) - 1
                distance = float(parts[2])
                
                # Update the corresponding sensor data
                self.sensor_data[sensor_index] = distance
                
                # Create and publish the message
                msg = Float32MultiArray()
                msg.data = [float(value) for value in self.sensor_data]
                self.publisher_.publish(msg)
                
                self.get_logger().info(f"Published sensor data: {msg.data}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to parse or publish data: {e}")

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
