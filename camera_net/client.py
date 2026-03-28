import yaml
from threading import Thread

import cv2
from cv2.typing import MatLike
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from camera_net_msgs.msg import CameraServ

img: list[MatLike] = []

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_client')
        self.get_logger().info('Starting Camera Client')
        
        # Parameters
        self.declare_parameter('camera', 'camera_server')
        self.declare_parameter('image', 'image_raw')
        self.declare_parameter('camera_info_url', '')
        
        # Get Parameters
        self.camera_param = self.get_parameter_or('camera').get_parameter_value().string_value
        self.image_param = self.get_parameter_or('image').get_parameter_value().string_value
        self.camera_info_url = self.get_parameter_or('camera_info_url').get_parameter_value().string_value
        
        # Subscriptions
        self.create_subscription(CameraServ, self.camera_param, self._camera, 10)
        
        # Publishers
        self.image = self.create_publisher(Image, self.image_param, 30)
        self.camera = self.create_publisher(CameraInfo, 'camera_info', 30)
        
        self.camera_info = CameraInfo()
        self.server = CameraServ()
        
    def _camera(self, msg: CameraServ):
        self.server = msg
        self.camera_info.height = msg.height
        self.camera_info.width = msg.width
        self.getCalibrationFile()

    def getCalibrationFile(self):
        if self.camera_info_url:
            node.get_logger().info('Loading Camera Calibration')
            with open(self.camera_info_url, 'r') as file:
                yaml_file = yaml.safe_load(file)
                
            self.camera_info.distortion_model = yaml_file['distortion_model']
            self.camera_info.d = yaml_file['distortion_coefficients']['data']
            self.camera_info.k = yaml_file['camera_matrix']['data']
            self.camera_info.r = yaml_file['rectification_matrix']['data']
            self.camera_info.p = yaml_file['projection_matrix']['data']

def get_stream():
    global img
    url = f'http://{node.server.address}:{node.server.port}'
    node.get_logger().info(f'Found camera at {url}')

    capture = cv2.VideoCapture(url)
    capture.set(cv2.CAP_PROP_FPS, 30)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G') )

    while capture.isOpened():
        success, frame = capture.read()

        # if frame is not read correctly ret is True
        if not success:
            node.get_logger().error('Can\'t receive frame (stream end?). Exiting ...')
            break

        # Rotate Image
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)

        img.append(frame)
        
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
    
    capture.release()
    cv2.destroyAllWindows()

def main():
    global node, img
    rclpy.init()
    node = CameraNode()
    
    # wait for camera communication
    while not node.server.port:
        rclpy.spin_once(node)

    streaming = Thread(target=get_stream)
    streaming.start()

    while True:
        br = CvBridge()
        if len(img) > 0:
            img_msg = br.cv2_to_imgmsg(img.pop(0), "bgr8")
            node.image.publish(img_msg)
            
            # Publish camera_info
            node.camera.publish(node.camera_info)
        
if __name__ == '__main__':
    main()