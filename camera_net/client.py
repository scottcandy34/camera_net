import os
os.environ['OPENCV_FFMPEG_LOGLEVEL'] = '-8'

import time
import signal
import yaml
from socket import create_connection
from threading import Event, Thread

import cv2
from cv2.typing import MatLike
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from camera_net_msgs.msg import CameraServ

img: list[MatLike] = []
stop = Event()

def requestStop(signum, frame):
    stop.set()
    os._exit(0)

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

def portOpen(url):
    # http://host:port
    host_port = url.split('://', 1)[-1]
    host, port = host_port.rsplit(':', 1)
    try:
        with create_connection((host, int(port)), timeout=0.5):
            return True
    except OSError:
        return False

def get_stream():
    global img
    capture = None
    url = None
    connected = False

    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except Exception:
        pass

    while not stop.is_set() and rclpy.ok():
        server = node.server
        if not server.port:
            if capture is not None:
                capture.release()
                capture = None
                url = None
            if connected:
                node.get_logger().warning('Camera disconnected')
                connected = False
            if stop.wait(0.5):
                break
            continue

        next_url = f'http://{server.address}:{server.port}'
        if capture is None or not capture.isOpened() or next_url != url:
            if connected:
                node.get_logger().warning('Camera disconnected')
                connected = False
            if capture is not None:
                capture.release()
                capture = None
            if not portOpen(next_url):
                url = None
                if stop.wait(0.5):
                    break
                continue
            url = next_url
            capture = cv2.VideoCapture(url)
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)
            capture.set(cv2.CAP_PROP_FPS, 30)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            if not capture.isOpened():
                capture.release()
                capture = None
                url = None
                if stop.wait(0.5):
                    break
                continue

        success, frame = capture.read()

        # if frame is not read correctly ret is True
        if not success:
            if connected:
                node.get_logger().warning('Camera disconnected')
                connected = False
            capture.release()
            capture = None
            url = None
            if stop.wait(0.5):
                break
            continue

        if not connected:
            node.get_logger().info(f'Connected to camera {url}')
            connected = True

        # Rotate Image
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)

        img.append(frame)

    if capture is not None:
        capture.release()
    cv2.destroyAllWindows()

def main():
    global node, img
    rclpy.init()
    signal.signal(signal.SIGINT, requestStop)
    signal.signal(signal.SIGTERM, requestStop)
    node = CameraNode()
    
    # wait for camera communication
    while not stop.is_set() and rclpy.ok() and not node.server.port:
        rclpy.spin_once(node, timeout_sec=0.1)

    streaming = Thread(target=get_stream, daemon=True)
    streaming.start()

    br = CvBridge()
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(img) > 0:
                img_msg = br.cv2_to_imgmsg(img.pop(0), "bgr8")
                node.image.publish(img_msg)
                
                # Publish camera_info
                node.camera.publish(node.camera_info)
    except KeyboardInterrupt:
        stop.set()
    os._exit(0)
        
if __name__ == '__main__':
    main()