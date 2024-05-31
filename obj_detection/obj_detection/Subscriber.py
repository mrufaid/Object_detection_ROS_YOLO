import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.pt")  # load a pretrained model 
def det_obj(img):
    results = model(img,show=True)  

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'images',  # Change 'image_topic' to the actual topic name
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Display the image
            detected_obj= det_obj(cv_image)
            cv2.imshow('Image Subscriber', detected_obj)
            cv2.waitKey(1)  # Refresh display
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
