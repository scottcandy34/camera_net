import rclpy, cv2, time
from rclpy.node import Node
from flask import Flask, Response
from socket import socket
from camera_net_msgs.msg import CameraServ
import netifaces
from threading import Thread

app = Flask(__name__)

node: Node = None
port: int
address: str
height: int
width: int

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

def getCamera():
    global camera, height, width
    # Compile GStreamer string
    gs_pipeline = "v4l2src ! video/x-raw, format=YUY2, width=640, height=480 ! v4l2convert ! video/x-raw,format=BGR ! appsink sync=false"
    
    # OpenCV Capture
    # camera = cv2.VideoCapture(gs_pipeline, cv2.CAP_GSTREAMER)
    camera = cv2.VideoCapture(0, cv2.CAP_ANY)
    
    if not camera.isOpened():
        node.get_logger().error("Cannot open camera")
        
    codec = 0x47504A4D # MJPG
    camera.set(cv2.CAP_PROP_FPS, 30.0)
    camera.set(cv2.CAP_PROP_FOURCC, codec)
    
    if height: camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if width: camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    
    # camera.release()
    # cv2.destroyAllWindows()
        
def getStream():
    while camera.isOpened():
        success, frame = camera.read()
        
        if not success:
            node.get_logger().warning("Failed to read frame from camera")
            break
        
        ret, encodedFrame = cv2.imencode('.jpg', frame)
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
        
        # Get Parameters
        self.height = self.get_parameter_or('height').get_parameter_value().integer_value
        self.width = self.get_parameter_or('width').get_parameter_value().integer_value
        
        # Publishers
        self.image = self.create_publisher(CameraServ, 'camera_server', 10)
        
        # Timer loops
        self.create_timer(0.1, self._actions)
    
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
    global port, address, node, height, width
    rclpy.init(args=args)
    node = CameraNode()
    
    height = node.height
    width = node.width

    # Set ip addr and port #
    port = getPort()
    address = getIP()
    
    # Configure Threads
    ros = Thread(target=startROS)
    flask = Thread(target=startFlask)
    
    getCamera()
    if not camera.isOpened():
        exit(-1)

    # Start threads
    ros.start()
    time.sleep(1)
    flask.start()
    
if __name__ == '__main__':
    main()