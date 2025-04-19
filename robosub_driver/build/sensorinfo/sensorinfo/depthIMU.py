import board
import adafruit_icm20x
import ms5837
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import threading
import time

msg = Float64MultiArray()
angle_scaled = 0.0

class DepthIMUNode(Node):
    def __init__(self):
        super().__init__('depthimu_node')
        
        # Create a publisher for sensor data
        self.sensor_publisher = self.create_publisher(Float64MultiArray, 'sensor_data', 10)

        # Create a timer to read both sensors at a 1 Hz rate
        #time.sleep(2)
        self.timer = self.create_timer(0.1, self.publish_data)
    
    def publish_data(self):
        self.sensor_publisher.publish(msg)
        #self.get_logger().error("Published Data to: 'sensor topic'")
        print(msg.data[0],end= "  ")
        print(msg.data[1])

class Interpreter(Node):
    def __init__(self):
        super().__init__('scaled_angle')

        # Initialize the Depth Sensor (MS5837)
        self.depth_sensor = ms5837.MS5837_30BA()
        if not self.depth_sensor.init():
            self.get_logger().error("Depth sensor could not be initialized")
            raise RuntimeError("Depth sensor initialization failed")
        self.depth_sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        
        # Initialize the IMU Sensor (ICM20649)
        i2c = board.I2C()
        self.imu_sensor = adafruit_icm20x.ICM20649(i2c)
        
        self.timer = self.create_timer(0.0000025, self.read_data)
    
    def read_data(self):
        global angle_scaled 
        

        try:
            # Depth Sensor Readings
            depth, pressure, temperature = None, None, None
            if self.depth_sensor.read():
                # depth = self.depth_sensor.depth()
                pressure = self.depth_sensor.pressure(ms5837.UNITS_mbar)
                temperature = self.depth_sensor.temperature(ms5837.UNITS_Centigrade)
                depth = (pressure - 1050) / 80
            else:
                self.get_logger().error("Failed to read depth sensor")
            
            # IMU Sensor Readings
            acceleration = None
            gyro = None
            acceleration = self.imu_sensor.acceleration  # Tuple (X, Y, Z) in m/s^2
            gyro = self.imu_sensor.gyro  # Tuple (X, Y, Z) in rad/s
            angle_scaled = (gyro[2]*3.3333) + angle_scaled - 0.0019 #this weird number is here for drift from gyro
        
            try:
                
                #angle = [sum(pair) for pair in zip(gyro, angle)]
                # angle = angle + gyro
                # float(angle)
                acceleration = tuple(float(round(a, 2)) for a in acceleration)
                #gyro = tuple(float(round(g, 2)) for g in gyro)
            except Exception as e:
                self.get_logger().error(f"Failed to read IMU sensor: {e}")

            
            msg.data = [depth] + [angle_scaled]
            #self.get_logger().info(f"Received Sensor Data - Depth: {depth:.2f}")  #, Acceleration: {acceleration}, Gyro: {gyro}")
            #self.get_logger().info(f"Calculated Angle: {angle_scaled}")
        except Exception as e:
            self.get_logger().error(f"Error reading sensors: {e}")
        
        


def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    reader_node = Interpreter()
    time.sleep(2)
    depthimu_node = DepthIMUNode()
    
    
    # Create a MultiThreadedExecutor and add both nodes
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(reader_node)
    executor.add_node(depthimu_node)
    
    
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        depthimu_node.get_logger().info("Shutting down combined sensor node")
    finally:
        depthimu_node.destroy_node()
        reader_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
