import ms5837
import rclpy
import smbus2
from rclpy.node import Node
from std_msgs.msg import Float64
import os


class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')
        
        # Create an instance of the MS5837 class
        self.sensor = ms5837.MS5837_30BA()  # For Bar30 sensor
        # self.sensor = ms5837.MS5837_02BA()  # For Bar02 sensor
        
        # Initialize the sensor
        if not self.sensor.init():
            self.get_logger().error("Sensor could not be initialized")
            raise RuntimeError("Sensor initialization failed")
        
        # Set fluid density to that of seawater (kg/m^3)
        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)  # Use ms5837.DENSITY_FRESHWATER for fresh water
        
        # Create a publisher for depth data
        self.depth_publisher = self.create_publisher(Float64, 'depthm', 10)
        
        # Open a file to log the depth data
        home_directory = os.path.expanduser("~")
        self.log_file_path = os.path.join(home_directory, "depth_log.txt")
        self.log_file = open(self.log_file_path, "w")
        
        # Create a timer to periodically read sensor data
        self.timer = self.create_timer(1.0, self.read_sensor_data)  # 1 Hz
        
    def read_sensor_data(self):
        if self.sensor.read():
            depth = self.sensor.depth()
            pressure = self.sensor.pressure(ms5837.UNITS_mbar)
            temperature = self.sensor.temperature(ms5837.UNITS_Centigrade)
            
            # Log the data
            self.get_logger().info(f"Depth: {depth:.2f} m, Pressure: {pressure:.2f} mbar, Temperature: {temperature:.2f} C")
            
            # Publish the depth
            depth_msg = Float64()
            depth_msg.data = depth
            self.depth_publisher.publish(depth_msg)
            
            # Write to the log file
            self.log_file.write(f"Depth: {depth:.2f} m, Pressure: {pressure:.2f} mbar, Temperature: {temperature:.2f} C\n")
        else:
            self.get_logger().error("Sensor read failed")
    
    def destroy_node(self):
        # Close the log file
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    depth_sensor_node = DepthSensorNode()
    
    try:
        rclpy.spin(depth_sensor_node)
    except KeyboardInterrupt:
        depth_sensor_node.get_logger().info("Shutting down node")
    finally:
        depth_sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

