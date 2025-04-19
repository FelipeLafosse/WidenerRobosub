import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data  # Correct QoS for images

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters with default values
        # Parameters for edge detection area range
        self.declare_parameter('min_area', 25000)
        self.declare_parameter('max_area', 50000)
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.declare_parameter('frequency', 8)
        self.declare_parameter('gstreamer_pipeline', 'v4l2src ! video/x-raw, width=640, height=480 ! videoconvert ! appsink')

        # Get the parameter values
        self.frequency = self.get_parameter('frequency').value
        self.gstreamer_pipeline = self.get_parameter('gstreamer_pipeline').value

        # ROS2 Publishers
        self.color_pub = self.create_publisher(String, 'color_topic', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)  # Image publisher
        self.edge_pub = self.create_publisher(Image, 'camera/edge', qos_profile_sensor_data)

        # Timer
        self.timer = self.create_timer(1.0 / self.frequency, self.process_frame)

        # OpenCV Video Capture with GStreamer pipeline
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream with GStreamer pipeline")
            raise RuntimeError("Video capture initialization failed")

        # Camera resolution (optional to retrieve)
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera resolution set to {width}x{height}")

        # Define color ranges (BGR format)
        self.color_ranges = {
            "red": ([9, 13, 112], [41, 35, 171]),
            "green": ([95, 100, 29], [160, 181, 109]),
            "blue": ([90, 50, 20], [150, 120, 80]),
            "orange": ([7, 40, 136], [49, 104, 193]),
            "yellow": ([10, 100, 134], [46, 152, 186]),
            "purple": ([80, 30, 40], [152, 84, 128]),
            "black": ([24, 19, 17], [70, 70, 60]),
        }
        

        self.bridge = CvBridge()  # OpenCV to ROS2 image conversion


    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame")
            return

        # --- Color Detection ---
        detected_color = "None"
        max_reading = 0
        dem = 255 * frame.shape[0] * frame.shape[1]
        thresh = 0.2

        for color_name, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(frame, np.array(lower), np.array(upper))
            color_t = np.sum(mask) / dem
            self.get_logger().info(f'{color_name.capitalize()}: {color_t:.3f}')

            if color_t > max_reading:
                max_reading = color_t
                detected_color = color_name

        if max_reading > thresh:
            self.get_logger().info(f"Detected {detected_color} with reading {max_reading:.3f}")
        else:
            detected_color = "None"

        self.color_pub.publish(String(data=detected_color))

        # --- Edge Detection ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(blur, 100, 200)

        # --- ROS2 Image Publishing ---
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)

            edge_image = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
            self.edge_pub.publish(edge_image)

        except Exception as e:
            self.get_logger().error(f"Failed to convert/publish images: {str(e)}")

        # Find contours from the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        frame_found = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                if len(approx) == 4:  # Detecting a rectangular shape (4 corners)
                    frame_found = True
                    break

        # If an object (frame) is detected, publish to the edge topic
        if frame_found:
            self.edge_pub.publish(String(data='go_forward'))
            self.get_logger().info("Object (frame) detected, publishing 'go_forward' command.")

        # Convert the image to ROS2 Image message and publish
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert/publish image: {str(e)}")

        # Optional: Visual Debug View
        frame_display = cv2.rotate(frame, cv2.ROTATE_180)
        edge_display = cv2.rotate(edges, cv2.ROTATE_180)

        cv2.imshow('Raw Frame', frame_display)
        cv2.imshow('Edge Detection', edge_display)

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
