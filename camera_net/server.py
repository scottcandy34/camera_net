import time
import os
import signal
import netifaces
from socket import socket
from threading import Event, Lock, Thread

import cv2
import rclpy
from rclpy.node import Node
from flask import Flask, Response

from camera_net_msgs.msg import CameraServ

app = Flask(__name__)

node: Node = None
port: int
address: str
height: int
width: int
jpeg_quality: int = 60
target_fps: float = 15.0
stop = Event()
camera = None
opened_path = None
latest = None
frame_id = 0
camera_lock = Lock()

def requestStop(signum, frame):
    stop.set()
    os._exit(0)

def getIP():
    deviceNames = netifaces.interfaces()
    for name in deviceNames:
        info = netifaces.ifaddresses(name)
        if len(info) == 3:
            addr = info[netifaces.AF_INET][0]['addr']
            if addr != '127.0.0.1' and addr != '192.168.186.3':
                return addr
    
    node.get_logger().error("not connected to network")
    exit(-1)

def getPort() -> int:
    with socket() as s:
        s.bind(('', 0))
        return s.getsockname()[1]

def captureDevices():
    primary = []
    other = []
    try:
        names = os.listdir('/dev')
    except OSError:
        return primary
    for name in sorted(names):
        if not name.startswith('video') or not name[5:].isdigit():
            continue
        path = f'/dev/{name}'
        index_file = f'/sys/class/video4linux/{name}/index'
        try:
            with open(index_file) as handle:
                node_index = handle.read().strip()
        except OSError:
            node_index = '0'
        if node_index == '0':
            primary.append(path)
        else:
            other.append(path)
    return primary or other

def openCamera():
    global camera, height, width, opened_path
    # Compile GStreamer string
    gs_pipeline = "v4l2src ! video/x-raw, format=YUY2, width=640, height=480 ! v4l2convert ! video/x-raw,format=BGR ! appsink sync=false"

    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except Exception:
        pass

    found = None
    with camera_lock:
        camera = cv2.VideoCapture()
        for path in captureDevices():
            if stop.is_set():
                break
            if camera.open(path, cv2.CAP_V4L2):
                found = path
                opened_path = path
                break
            camera.release()
        if found:
            codec = 0x47504A4D # MJPG
            camera.set(cv2.CAP_PROP_FPS, float(target_fps))
            camera.set(cv2.CAP_PROP_FOURCC, codec)
            if height: camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if width: camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))

    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_WARNING)
    except Exception:
        pass
    return found

def watchCamera():
    global latest, frame_id
    missing_logged = False
    had_frame = False
    min_interval = 1.0 / max(float(target_fps), 1.0)
    last = 0.0
    while not stop.is_set():
        with camera_lock:
            open_now = camera is not None and camera.isOpened()
            cam = camera
            path = opened_path
        if not open_now:
            latest = None
            if not missing_logged:
                if had_frame:
                    node.get_logger().warning('Camera disconnected')
                    had_frame = False
                node.get_logger().warning('No camera found')
                missing_logged = True
            openCamera()
            if stop.wait(0.5):
                break
            continue

        now = time.monotonic()
        if now - last < min_interval:
            if stop.wait(0.005):
                break
            continue

        success, frame = cam.read()
        if not success:
            with camera_lock:
                if camera is cam and camera is not None:
                    camera.release()
            latest = None
            continue

        last = now
        if not had_frame:
            node.get_logger().info(f'Connected to camera {path}')
            had_frame = True
            missing_logged = False
        latest = frame
        frame_id += 1

def getStream():
    seen = -1
    while not stop.is_set():
        current = frame_id
        frame = latest
        if frame is None or current == seen:
            if stop.wait(0.01):
                break
            continue
        seen = current

        ret, encodedFrame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
        if not ret:
            continue
        imgBytes = encodedFrame.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + imgBytes + b'\r\n')

        if cv2.waitKey(1) == ord('q'):
            break
  
@app.route('/')
def video_stream():
    return Response(getStream(), mimetype='multipart/x-mixed-replace; boundary=frame')

def startFlask():
    app.run(port=port, host=address)
    node.get_logger().error('Flask stopped running')
    
class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_server')
        self.get_logger().info('Starting Camera Server')
        
        # Parameters
        self.declare_parameter('height', 480)
        self.declare_parameter('width', 640)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('fps', 15.0)
        
        # Get Parameters
        self.height = self.get_parameter_or('height').get_parameter_value().integer_value
        self.width = self.get_parameter_or('width').get_parameter_value().integer_value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.fps = float(self.get_parameter('fps').value)
        
        # Publishers
        self.image = self.create_publisher(CameraServ, 'camera_server', 10)
        
        # Timer loops
        self.create_timer(1.0, self._actions)
    
    def _actions(self):
        msg = CameraServ()
        msg.address = address
        msg.port = port
        msg.height = self.height
        msg.width = self.width
        
        self.image.publish(msg)

def startROS():
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    node.get_logger().error('ROS stopped running')

def main(args=None):
    global port, address, node, height, width, jpeg_quality, target_fps
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, requestStop)
    signal.signal(signal.SIGINT, requestStop)
    node = CameraNode()
    
    height = node.height
    width = node.width
    jpeg_quality = node.jpeg_quality
    target_fps = node.fps

    # Set ip addr and port #
    port = getPort()
    address = getIP()
    
    # Configure Threads
    ros = Thread(target=startROS, daemon=True)
    flask = Thread(target=startFlask, daemon=True)
    watch = Thread(target=watchCamera, daemon=True)

    # Start threads. Camera search keeps going until one is found.
    ros.start()
    watch.start()
    time.sleep(1)
    flask.start()

    # Stay on the main thread so SIGTERM/SIGINT can stop the process.
    stop.wait()
    os._exit(0)
    
if __name__ == '__main__':
    main()