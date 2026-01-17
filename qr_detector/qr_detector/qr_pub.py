import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2

class QRPublisher(Node):
    def __init__(self):
        super().__init__('qr_publisher')

        self.publisher = self.create_publisher(String, 'qr_data', 10)

        self.cap = cv2.VideoCapture(0)
        self.detector = cv2.QRCodeDetector()

        self.timer = self.create_timer(0.05, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        data, points, _ = self.detector.detectAndDecode(frame)

        if data:
            msg = String()
            msg.data = data
            self.publisher.publish(msg)
            self.get_logger().info(f"Published QR: {data}")

        cv2.imshow("QR Scanner", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = QRPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
