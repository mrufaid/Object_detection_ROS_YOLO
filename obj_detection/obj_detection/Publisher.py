import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def find_webcam_indices():
    webcam_indices = []
    for i in range(10):  # Try up to 10 indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Webcam found at index {i}")
            webcam_indices.append(i)
            cap.release()
    if not webcam_indices:
        print("No webcams found")
    return webcam_indices

# Find the indices of all available webcams
webcam_indices = find_webcam_indices()
print(webcam_indices)

class img_publishing(Node): 
    def __init__(self):
        super().__init__("img_pub")
        self.publisher=self.create_publisher(Image,'images',10)
        self.timer=self.create_timer(0.1,self.img_callback)
        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(webcam_indices[0])
        self.get_logger().info('node started')

    def img_callback(self):
        ret, frame = self.cap.read()
        if ret==True:
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(ros_image)
        else:
            self.get_logger().info('frame not read')

        
            



 
 
def main(args=None):
    rclpy.init(args=args)
    node = img_publishing() 
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()