import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float64MultiArray, String, Int64MultiArray, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

msg = Float64MultiArray()

# -------- DEPTH IMU NODE -------- #
class DepthIMU(Node):
    def __init__(self):
        super().__init__('depthimu_sub')
        self.sensor_subscription = self.create_subscription(Float64MultiArray, 'sensor_data', self.sensor_callback, 10)

    def sensor_callback(self, msg):
        global depth
        global angle
        depth = msg.data[0]
        angle = msg.data[1]

# -------- HYDROPHONE NODE -------- #
class Hydrophone(Node):
    def __init__(self):
        super().__init__('hydrophone_sub')
        self.hydro_sub = self.create_subscription(Float32, 'ping_detected', self.ping_callback, 10)

    def ping_callback(self, msg):
        ping = msg.data
        self.get_logger().info(f"Received ping: {ping}")

# -------- CAMERA COLOR CALLBACK -------- #
class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.color_sub = self.create_subscription(String, 'color_topic', self.camera_callback, 10)

    def camera_callback(self, msg):
        color = msg.data
        color_dir = self.get_motor_direction(color)
        self.get_logger().info(f"Received color: {color}, Setting AUV direction to: {color_dir}")

    def get_motor_direction(self, color):
        color = color.lower().strip()

        direction = "none"
        if "green" in color:
            direction = 1
        elif "red" in color:
            direction = 2
        elif "orange" in color:
            direction = 3
        elif "purple" in color:
            direction = 0
        else:
            self.get_logger().warn(f"Unrecognized color: '{color}'")

        return direction

    #def get_motor_direction(self, color):
    #    speed_map = {
    #        "blue": "forward",
    #        "red": "backward",
    #        "yellow": "rot_180"
    #    }
    #    return speed_map.get(color, "none")

# -------- EDGE DETECTOR NODE -------- #
class EdgeDetectorNode(Node):
    def __init__(self):
        super().__init__('edge_detector_sub')

        self.image_sub = self.create_subscription(String, 'edge_detect', self.edge_callback, 10)
        self.get_logger().info("EdgeDetectorNode initialized and listening for edge images.")

    def edge_callback(self, msg):
        if msg.data == 'rectangle_detected':
            self.get_logger().info("moving forward")
            direction = 1
        else:
            direction = 0
            return direction

class Publishing_node(Node):
    def __init__(self):
        super().__init__('publish_motorcommand')
        self.publisher = self.create_publisher(Float64MultiArray, 'drive_topic', 10)
        self.timer = self.create_timer(0.2, self.publish)

    def publish(self):
        global depth
        global current_depth
        global angle 
        global heading
        global direction
        global speed
        msg.data = [depth] + [current_depth] + [angle] + [heading] + [direction] + [speed]
        self.publisher.publish(msg)
        #print('publsih')

# -------- MOTOR COMMAND NODE -------- #
class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_node')
        self.publisher = self.create_publisher(Float64MultiArray, 'drive_topic', 10)
        self.get_logger().info("Motor Command Node Initialized")
        self.timer = self.create_timer(0.5, self.run)

    def run(self):
        global depth
        global current_depth
        global angle 
        global heading
        global direction
        global speed

        mode = input('enter depth, heading direction or speed: ')
        if mode == 'depth':
            depth = float(input('enter a new depth: '))
        elif mode =='heading':
            heading = float(input('enter a new heading (0 is forward): '))
        elif mode == 'direction':
            direction = input('stop, forward, backward, left, right: ')
            if direction == 'stop':
                direction = 0
            if direction == 'forward':
                direction = 1
            if direction == 'backward':
                direction = 2
            if direction == 'left':
                direction = 3
            if direction == 'right':
                direction = 4
        elif mode == 'speed':
            speed = float(input('enter a new speed [0-100]: '))
        else:
            mode = 'none'
            print('enter a right mode idiot...')

# -------- MAIN -------- #
def main():
    rclpy.init()

    motor_command_publisher = MotorCommandPublisher()
    publishing_something = Publishing_node()
    depth_imu = DepthIMU()
    hydrophone = Hydrophone()
    camera = Camera()
    edge_detector = EdgeDetectorNode()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(publishing_something)
    executor.add_node(motor_command_publisher)
    executor.add_node(depth_imu)
    executor.add_node(hydrophone)
    executor.add_node(camera)
    executor.add_node(edge_detector)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motor_command_publisher.destroy_node()
        depth_imu.destroy_node()
        hydrophone.destroy_node()
        camera.destroy_node()
        edge_detector.destroy_node()
        publishing_something.destroy_node()
        rclpy.shutdown()

depth = 0.0
current_depth = 0.0
angle = 0.0
heading = 0.0
direction = 0.0
speed = 0.0

if __name__ == '__main__':
    main()
