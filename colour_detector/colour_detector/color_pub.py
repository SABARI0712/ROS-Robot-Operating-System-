import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class ColorPublisher(Node):
    def __init__(self):
        super().__init__('color_publisher')
        self.publisher = self.create_publisher(String, 'detected_color', 10)

        # open camera
        self.cap = cv2.VideoCapture(0)

        # HSV ranges (from your original code)
        self.lower_red1 = np.array([0,120,70])
        self.upper_red1 = np.array([10,255,255])
        self.lower_red2 = np.array([170,120,70])
        self.upper_red2 = np.array([180,255,255])

        self.lower_green = np.array([36,50,70])
        self.upper_green = np.array([89,255,255])

        self.lower_blue = np.array([94,80,2])
        self.upper_blue = np.array([126,255,255])

        # run at ~20 fps
        self.timer = self.create_timer(0.05, self.process_frame)

    def detect_color(self, frame, mask, name):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 150:
                return name
        return None

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
                   cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        blue_mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        detected = None
        for mask, name in [(red_mask, "RED"), (green_mask, "GREEN"), (blue_mask, "BLUE")]:
            result = self.detect_color(frame, mask, name)
            if result:
                detected = result
                break

        if detected:
            msg = String()
            msg.data = detected
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {detected}')

        cv2.imshow("Color Detection", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ColorPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
