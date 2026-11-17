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
latest_jpeg = None
frame_id = 0
reads_ok = 0
in_read = False
dropped = False
frame_kind = 'encoded'
use_gstreamer = False
camera_lock = Lock()
failed_until = {}
last_reset = {}
reset_logged = False

def usbDevice(video):
    name = os.path.basename(video)
    return os.path.dirname(os.path.realpath(f'/sys/class/video4linux/{name}/device'))

def resetUsb(video):
    global reset_logged
    if not video:
        return False
    try:
        dev = usbDevice(video)
    except OSError:
        return False
    now = time.monotonic()
    if now - last_reset.get(dev, 0) < 30.0:
        return False
    auth = os.path.join(dev, 'authorized')
    try:
        with open(auth, 'w') as handle:
            handle.write('0\n')
        time.sleep(0.4)
        with open(auth, 'w') as handle:
            handle.write('1\n')
    except OSError as err:
        if not reset_logged:
            node.get_logger().error(f'USB reset failed for {video}: {err}')
            reset_logged = True
        return False
    last_reset[dev] = now
    node.get_logger().warning(f'Reset USB camera {video}')
    return True

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

def jpegBytes(frame):
    if frame is None:
        return None
    raw = frame.tobytes()
    if len(raw) < 4 or raw[0] != 0xFF or raw[1] != 0xD8:
        return None
    end = raw.rfind(b'\xff\xd9')
    if end == -1:
        return raw
    return raw[:end + 2]

def applyCapture(cap):
    global height, width
    cap.set(cv2.CAP_PROP_FOURCC, 0x47504A4D) # MJPG
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or int(width)
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or int(height)
    width = got_w
    height = got_h

def openGStreamer(path):
    pipeline = (
        f"v4l2src device={path} ! "
        f"image/jpeg,width={int(width)},height={int(height)},framerate={int(target_fps)}/1 ! "
        f"appsink drop=1 max-buffers=1 sync=false"
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        cap.release()
        return None
    ok, frame = cap.read()
    if not ok or jpegBytes(frame) is None:
        cap.release()
        return None
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or int(width)
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or int(height)
    return cap, got_w, got_h

def openCamera():
    global camera, height, width, opened_path, use_gstreamer

    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except Exception:
        pass

    found = None
    chosen = None
    now = time.monotonic()
    for path in captureDevices():
        if stop.is_set():
            break
        if failed_until.get(path, 0) > now:
            continue
        if use_gstreamer:
            opened = openGStreamer(path)
            if opened is None:
                use_gstreamer = False
            else:
                cap, got_w, got_h = opened
                chosen = (cap, path, got_w, got_h)
                break
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        applyCapture(cap)
        chosen = (cap, path, width, height)
        break

    with camera_lock:
        if chosen is None:
            camera = None
        else:
            cap, path, got_w, got_h = chosen
            camera = cap
            opened_path = path
            width = got_w
            height = got_h
            found = path

    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except Exception:
        pass
    return found

def jpegSize(data):
    i = 2
    n = len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            h = int.from_bytes(data[i + 5:i + 7], 'big')
            w = int.from_bytes(data[i + 7:i + 9], 'big')
            if w and h:
                return w, h
            return None
        if marker == 0xD8:
            i += 2
            continue
        if marker in (0xD9, 0xDA):
            return None
        if i + 4 > n:
            return None
        seg = int.from_bytes(data[i + 2:i + 4], 'big')
        if seg < 2:
            return None
        i += 2 + seg
    return None

def watchCamera():
    global latest_jpeg, frame_id, frame_kind, width, height, camera, reads_ok, in_read, dropped
    missing_logged = False
    had_frame = False
    misses = 0
    fail_since = 0.0
    min_interval = 1.0 / max(float(target_fps), 1.0)
    last = 0.0
    while not stop.is_set():
        with camera_lock:
            open_now = camera is not None and camera.isOpened()
            cam = camera
            path = opened_path
        if not open_now:
            latest_jpeg = None
            if not missing_logged:
                if had_frame:
                    node.get_logger().warning('Camera disconnected')
                    had_frame = False
                node.get_logger().warning('No camera found')
                missing_logged = True
            openCamera()
            misses = 0
            if stop.wait(0.5):
                break
            continue

        with camera_lock:
            in_read = True
        success, frame = cam.read()
        with camera_lock:
            in_read = False
            was_dropped = dropped
            if was_dropped:
                dropped = False
                camera = None
        if was_dropped:
            latest_jpeg = None
            misses = 0
            fail_since = 0.0
            continue

        if not success:
            if not had_frame:
                with camera_lock:
                    if camera is cam and camera is not None:
                        camera.release()
                        camera = None
                latest_jpeg = None
                resetUsb(path)
                failed_until[path] = time.monotonic() + 5.0
                if stop.wait(0.2):
                    break
                continue
            if fail_since == 0.0:
                fail_since = time.monotonic()
            if time.monotonic() - fail_since < 10.0:
                if stop.wait(0.05):
                    break
                continue
            with camera_lock:
                if camera is cam and camera is not None:
                    camera.release()
                    camera = None
            latest_jpeg = None
            resetUsb(path)
            failed_until[path] = time.monotonic() + 5.0
            misses = 0
            fail_since = 0.0
            continue

        misses = 0
        fail_since = 0.0
        reads_ok += 1
        now = time.monotonic()
        if now - last < min_interval:
            continue
        last = now

        jpeg = jpegBytes(frame)
        if jpeg is None:
            ret, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
            if not ret:
                continue
            jpeg = encoded.tobytes()
            kind = 'encoded'
        else:
            kind = 'jpeg'

        last = now
        if not had_frame:
            if kind == 'jpeg':
                size = jpegSize(jpeg)
                if size:
                    width, height = size
            frame_kind = kind
            node.get_logger().info(f'Connected to camera {path} {int(width)}x{int(height)} {kind}')
            had_frame = True
            missing_logged = False
        latest_jpeg = jpeg
        frame_id += 1

def guardCamera():
    global dropped
    last_reads = -1
    last_change = time.monotonic()
    while not stop.is_set():
        if stop.wait(0.25):
            break
        with camera_lock:
            open_now = camera is not None and camera.isOpened()
            cam = camera
            busy = in_read
            count = reads_ok
        if not open_now or not busy or count == 0:
            last_reads = count
            last_change = time.monotonic()
            continue
        if count != last_reads:
            last_reads = count
            last_change = time.monotonic()
            continue
        if time.monotonic() - last_change < 10.0:
            continue
        with camera_lock:
            if camera is cam and camera is not None and in_read and not dropped:
                camera.release()
                dropped = True
        last_change = time.monotonic()

def getStream():
    seen = -1
    while not stop.is_set():
        current = frame_id
        jpeg = latest_jpeg
        if jpeg is None or current == seen:
            if stop.wait(0.01):
                break
            continue
        seen = current

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
  
@app.route('/')
def video_stream():
    return Response(getStream(), mimetype='multipart/x-mixed-replace; boundary=frame')

def startFlask():
    app.run(port=port, host='0.0.0.0', threaded=True, use_reloader=False)
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
        msg.height = int(height)
        msg.width = int(width)
        
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
    guard = Thread(target=guardCamera, daemon=True)

    # Start threads. Camera search keeps going until one is found.
    ros.start()
    watch.start()
    guard.start()
    time.sleep(1)
    flask.start()

    # Stay on the main thread so SIGTERM/SIGINT can stop the process.
    stop.wait()
    os._exit(0)
    
if __name__ == '__main__':
    main()