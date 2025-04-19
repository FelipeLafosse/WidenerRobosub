import time
import board
import adafruit_icm20x
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import subprocess

class IMUSensorNode(Node):
    def __init__(self):
        super().__init__('imu_sensor_node')
        
        # Initialize the ICM20649 sensor
        i2c = board.I2C()  # Uses board.SCL and board.SDA
        self.icm = adafruit_icm20x.ICM20649(i2c)
        
        # Create a publisher for IMU data
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Create a timer to periodically read sensor data
        self.timer = self.create_timer(0.5, self.publish_imu_data)  # 0.5-second interval
        
    def publish_imu_data(self):
        try:
            # Read data from the IMU
            acceleration = self.icm.acceleration  # Tuple (X, Y, Z) in m/s^2
            gyro = self.icm.gyro  # Tuple (X, Y, Z) in rad/s
            
            acceleration = tuple(float(round(a, 2)) for a in acceleration)
            gyro = tuple(float(round(g, 2)) for g in gyro)

            # Create and populate the Imu message
            imu_msg = Imu()
            
            # Populate linear acceleration
            imu_msg.linear_acceleration.x = acceleration[0]
            imu_msg.linear_acceleration.y = acceleration[1]
            imu_msg.linear_acceleration.z = acceleration[2]
            
            # Populate angular velocity
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            
            # Publish the message
            self.imu_publisher.publish(imu_msg)
            self.get_logger().info(f"Published IMU Data: Accel={acceleration}, Gyro={gyro}")

            # subprocess.Popen(["gnome-terminal","--","bash","-c","ros2 topic echo imu/data_raw | grep -E 'angular_velocity:|linear_acceleration:|x:|y:|z:'; exec bash"])

        except Exception as e:
            self.get_logger().error(f"Error reading sensor data: {e}")


def main(args=None):
    rclpy.init(args=args)
    imu_sensor_node = IMUSensorNode()
    
    try:
        rclpy.spin(imu_sensor_node)
    except KeyboardInterrupt:
        imu_sensor_node.get_logger().info("Shutting down IMU sensor node")
    finally:
        imu_sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
