# Author: Endri Dibra 

# Project: Mia Hand Grasp + Wrist Latency Benchmark System (Voice-Targeted Vision Version)

# Description: Integrating multi-threaded speech recognition, object detection 
# tracking, real-time depth measurements, and Dynamixel hardware control modules 
# to coordinate semi-autonomous upper limb prosthetic grasping procedures

# Importing network socket protocols
# to interface with low-level data servers
import socket

# Importing system tracking timers
# to measure streaming execution delays
import time

# Importing robotic operating framework
# library to manage node communication
import rclpy

# Importing multi-threaded control suites
# to handle concurrent processing layers
import threading 

# Importing kernel interrupt routines
# to capture system shutdown signals
import signal

# Importing operating system parameters
# to manage direct script terminations
import sys

# Importing numerical python library
# to handle frame matrix conversions
import numpy as np

# Importing open computer vision
# library for rendering image frames
import cv2

# Importing Intel RealSense SDK
# to interface with hardware devices
import pyrealsense2 as rs

# Importing audio processing tools
# to transcribe verbal voice commands
import speech_recognition as sr

# Importing real-time object detection
# architecture for spatial tracking layers
from ultralytics import YOLO

# Extracting computational node modules
# from core robotic framework libraries
from rclpy.node import Node

# Extracting blank system message forms
# from operational robotic service classes
from std_srvs.srv import Empty

# Extracting custom activation request structures
# from proprietary prosthetic manipulation packets
from mia_hand_msgs.srv import ExecuteGrasp

# Extracting physical motor control hooks
# from Dynamixel hardware developer toolkits
from dynamixel_sdk import *

# Mapping internal firmware hardware address registers
# to control torque activation and operating parameters
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11

# Setting communication criteria properties
# matching active actuator bus parameters
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0' 


# Constructing the master controller class
# to regulate physical peripheral behaviors
class MiaPhysicalController(Node):

    # Initializing operational software properties
    # to maintain system node tracking loops
    def __init__(self):

        # Invoking parent node configurations
        # to register peripheral controller labels
        super().__init__('mia_physical_controller')

        # Initializing thread lock primitives
        # to isolate concurrent data updates
        self.lock = threading.Lock()

        # Initializing pattern recognition indicators
        # to follow incoming control sequences
        self.last_received_id = None
        self.consecutive_count = 0
        self.required_confirmations = 2
        self.motion_start_time = None

        # Tracking state machine flags
        # to index operational behaviors
        self.system_state = 'IDLE' 
        self.suggested_grasp = 'cylindrical'  
        self.tracked_object_label = "None"
        self.tracked_object_dist = -1.0
        self.target_class = None  

        # Initializing physical interface lines
        # to connect external motor hardware
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Evaluating interface connectivity lines
        # to catch serial port errors
        if not self.port_handler.openPort():

            # Logging immediate connection warnings
            # inside runtime diagnostic pipelines
            self.get_logger().error("PORT OPEN FAILED")
            
            return

        # Evaluating data delivery rates
        # to catch communications setup errors
        if not self.port_handler.setBaudRate(BAUDRATE):

            # Logging hardware configuration failures
            # inside runtime diagnostic pipelines
            self.get_logger().error("BAUDRATE FAILED")
            
            return

        # Modifying control register inputs
        # to change internal operating profiles
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_OPERATING_MODE, 3)
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        # Enforcing physical step increments
        # and movement limit specifications
        self.STEP_SIZE = 200
        self.UPDATE_DELAY = 0.03
        self.MIN_POSITION = 700
        self.MAX_POSITION = 2500
        self.CENTER_POSITION = 1600

        # Establishing baseline structural bounds
        # and homing actuators on startup
        self.current_position = self.CENTER_POSITION
        self.move_wrist(self.CENTER_POSITION) 

        # Initializing boolean movement monitors
        # to verify active directional loops
        self.moving_left = False
        self.moving_right = False
        self.moving_center = False
        self.is_stopped = False

        # Formulating explicit service client hooks
        # targeting corresponding configuration profiles
        self.grasp_clients = {
           
            'cylindrical': self.create_client(ExecuteGrasp, '/mia_hand/grasps/cylindrical/execute'),
            'pinch': self.create_client(ExecuteGrasp, '/mia_hand/grasps/pinch/execute'),
            'lateral': self.create_client(ExecuteGrasp, '/mia_hand/grasps/lateral/execute'),
            'relax': self.create_client(ExecuteGrasp, '/mia_hand/grasps/relax/execute')
        }
       
        self.stop_client = self.create_client(Empty, '/mia_hand/stop')
        self.play_client = self.create_client(Empty, '/mia_hand/play')

        # Introducing safety startup delays
        # to let connection brokers clear
        time.sleep(0.5) 
       
        self.send_grasp('relax', time.perf_counter())

        # Pushing initial configuration notices
        # onto primary operational tracking logs
        self.get_logger().info("Mia Controller Ready (Default Position: Centered & Opened)")

    # Measuring structural time differentials
    # to evaluate internal operational latencies
    def log_latency(self, label, start):

        # Computing execution time spreads
        # converted into standard milliseconds
        t = (time.perf_counter() - start) * 1000

        # Printing calculated performance data
        # onto primary operational tracking logs
        self.get_logger().info(f"{label} LATENCY {t:.2f} ms")

    # Transmitting target angular commands
    # directly onto physical wrist motors
    def move_wrist(self, pulse):

        # Constraining target positioning demands
        # within allowed hardware operating margins
        pulse = max(self.MIN_POSITION, min(self.MAX_POSITION, pulse))

        # Pushing computed positioning bytes
        # onto target actuator control lines
        self.packet_handler.write4ByteTxRx(
         
            self.port_handler,
            DXL_ID,
            ADDR_GOAL_POSITION,
            pulse
        )

    # Fetching real time angle data
    # from physical hardware units
    def get_position(self):

        # Capturing direct bit responses
        # through linked telemetry checks
        pos, _, _ = self.packet_handler.read4ByteTxRx(
           
            self.port_handler,
            DXL_ID,
            ADDR_GOAL_POSITION
        )

        # Returning structural orientation results
        # back to requesting execution blocks
        return pos

    # Running concurrent looping mechanisms
    # to update active wrist orientation
    def wrist_movement_engine(self):

        # Sustaining structural rotation updates
        # while operational communication links persist
        while rclpy.ok():

            # Evaluating emergency termination switches
            # to suspend background execution flows
            if self.is_stopped:
           
                time.sleep(self.UPDATE_DELAY)
           
                continue

            # Preparing local variable placeholders
            # to store adjusted position metrics
            target_to_send = None

            # Isolating current hardware checks
            # inside isolated memory lock boundaries
            with self.lock:

                # Checking active negative offsets
                # to execute left rotation steps
                if self.moving_left:

                    # Verifying current positioning metrics
                    # stand above safety lower boundaries
                    if self.current_position > self.MIN_POSITION:
            
                        self.current_position -= self.STEP_SIZE
                        target_to_send = self.current_position
            
                    else:
            
                        self.moving_left = False
                        self.motion_start_time = None

                # Checking active positive offsets
                # to execute right rotation steps
                elif self.moving_right:

                    # Verifying current positioning metrics
                    # stand below safety upper boundaries
                    if self.current_position < self.MAX_POSITION:
                
                        self.current_position += self.STEP_SIZE
                        target_to_send = self.current_position
                
                    else:
                
                        self.moving_right = False
                        self.motion_start_time = None

                # Checking baseline centering flags
                # to execute tracking alignment steps
                elif self.moving_center:

                    # Formulating orientation threshold scales
                    # to measure absolute tracking errors
                    target = self.CENTER_POSITION
                    diff = self.current_position - target

                    # Snapping local position metrics
                    # if remaining distances hit step bounds
                    if abs(diff) <= self.STEP_SIZE:
                
                        self.current_position = target
                
                        target_to_send = self.current_position
                
                        self.moving_center = False
                        self.motion_start_time = None

                    # Reducing position metrics down
                    # if tracking signs indicate positive errors
                    elif diff > 0:
                        
                        self.current_position -= self.STEP_SIZE
                        target_to_send = self.current_position

                    # Boosting position metrics up
                    # if tracking signs indicate negative errors
                    else:
                    
                        self.current_position += self.STEP_SIZE
                        target_to_send = self.current_position

            # Transmitting calculated target steps
            # down to active tracking controllers
            if target_to_send is not None:
                
                self.move_wrist(target_to_send)

            # Pausing baseline loop updates
            # to match designated refreshing bounds
            time.sleep(self.UPDATE_DELAY)

    # Dispatched targeted closure configurations
    # toward remote robotic peripheral networks
    def send_grasp(self, grasp_type, start_time):

        # Validating current system constraints
        # before pushing external activation packets
        if self.is_stopped or grasp_type not in self.grasp_clients:
          
            return

        # Mapping physical closure percentages
        # matching specific finger geometry modes
        grasp_apertures = {
         
            'cylindrical': 85,
            'pinch': 85,
            'lateral': 55,
            'relax': 0
        }

        # Filling service payload criteria
        # to parameterize hand actuation paths
        req = ExecuteGrasp.Request()
        req.close_percent = grasp_apertures.get(grasp_type, 50)
        req.spe_for_percent = 30

        # Outputting target movement goals
        # inside diagnostic processing trackers
        self.get_logger().info(f"STARTING PHYSICAL {grasp_type.upper()}...")
       
        future = self.grasp_clients[grasp_type].call_async(req)

        # Handling async completion statuses
        # inside delegated callback blocks
        def callback(fut):
          
            try:
          
                response = fut.result()
                self.log_latency(f"PHYSICAL {grasp_type.upper()} COMPLETE", start_time)
          
            except Exception as e:
          
                self.get_logger().error(f"Grasp failed: {str(e)}")

        # Linking execution tracking tokens
        # onto active request futures
        future.add_done_callback(callback)

    # Halting structural subsystem interactions
    # to intercept physical hardware collisions
    def emergency_stop(self):

        # Blocking parallel thread changes
        # to rewrite control trajectories
        with self.lock:
        
            self.is_stopped = True
            self.moving_left = False
            self.moving_right = False
            self.moving_center = False

        # Freezing current spatial links
        # by repeating current measurements
        pos = self.get_position()
        self.move_wrist(pos)


# Processing verbal selection commands
# from external audio ingestion sockets
def run_voice_target_selection(node):

    # Initializing network communication ports
    # to host incoming audio strings
    voice_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    voice_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    voice_socket.bind(('0.0.0.0', 5001))
    voice_socket.listen(1)

    # Enforcing strict database filters
    # to filter allowed classification targets
    VALID_OBJECTS = {'bottle', 'cup'}

    # Sustaining audio stream tracking
    # while operational communication links persist
    while rclpy.ok():

        # Halting execution flow until
        # a remote packet client connects
        conn, _ = voice_socket.accept()

        # Isolating specific stream interfaces
        # inside active communication scopes
        with conn:

            # Capturing localized text words
            # passing across network sockets
            data = conn.recv(1024).decode().strip().lower()
            
            if not data:
            
                continue

            # Evaluating parsed input packets
            # within secure runtime locks
            with node.lock:

                # Intercepting master initialization flags
                # to clear running state variables
                if data == 'go':

                    # Inspecting operational state configurations
                    # to execute structural homing sequences
                    if node.system_state in ['IDLE', 'RELEASED_WAITING']:
                  
                        if node.system_state == 'RELEASED_WAITING':
                  
                            node.get_logger().info("Returning to default position: Centering wrist and opening hand...")
                  
                            node.moving_left = False
                            node.moving_right = False
                            node.moving_center = True 
                  
                            node.send_grasp('relax', time.perf_counter())
                        
                        # Resetting internal tracking profiles
                        # to drop preceding telemetry values
                        node.target_class = None
                
                        node.suggested_grasp = 'cylindrical'
                        node.tracked_object_label = "None"
                     
                        node.tracked_object_dist = -1.0
                        node.consecutive_count = 0
                
                        node.last_received_id = None
                        
                        # Shifting active workflow structures
                        # into target exploration flags
                        node.system_state = 'GET_TARGET'
                        node.get_logger().info("=== System State: GET_TARGET. Waiting for valid object name... ===")
                
                # Evaluating incoming string identifiers
                # while looking for target assignments
                elif node.system_state == 'GET_TARGET':

                    # Checking parsed name arrays
                    # against active model classes
                    if data in VALID_OBJECTS:
               
                        node.target_class = data
                        node.system_state = 'TRACKING'
                        node.get_logger().info(f"=== System State: TRACKING locked onto valid object [{data.upper()}] ===")

                    # Rejecting outside language inputs
                    # by showing explicit warning logs
                    else:
                  
                        node.get_logger().warn(
                            f"Rejected invalid object word: '{data.upper()}'. "
                            f"Allowed options are {list(VALID_OBJECTS)}. Remaining in GET_TARGET state..."
                        )


# Executing spatial tracking calculations
# inside dedicated frame processing pipelines
def vision_processing_thread(node):

    # Outputting active process statuses
    # inside diagnostic log paths
    node.get_logger().info("Vision Engine Idle. Waiting for configuration activation signal...")
    model = YOLO("yolo11n.pt")

    # Initializing hardware camera interfaces
    # to link spatial recording channels
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Attempting pipeline sensor connection
    # inside verified monitoring frames
    try:
  
        profile = pipeline.start(config)
  
    except Exception as e:
  
        node.get_logger().error(f"RealSense link failed: {e}")
  
        return

    # Extracting calibration metric factors
    # and tracking category configuration dictionaries
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    grasp_mapping = {'bottle': 'cylindrical', 'cup': 'cylindrical', 'can': 'cylindrical', 'apple': 'pinch', 'banana': 'lateral'}

    # Driving continuous visual calculations
    # while operational communication links persist
    try:
        
        while rclpy.ok():

            # Copying system state variables
            # inside isolated thread checkouts
            with node.lock:
        
                current_state = node.system_state
                active_filter = node.target_class

            # Fetching raw spatial data packets
            # from connected sensor stream pipelines
            frames = pipeline.wait_for_frames(5000)
        
            if not frames:
        
                continue

            # Splitting aggregate data packages
            # into separate color and depth layers
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
          
            if not depth_frame or not color_frame:
          
                continue

            # Converting raw graphic data arrays
            # into standard computer vision layouts
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Formulating visual representation feed views
            # using thermal color projection matrices
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)

            # Intercepting inactive system conditions
            # to pass standard graphic indicators
            if current_state != 'TRACKING':
            
                cv2.putText(color_image, f"SYSTEM STATE: {current_state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
                if active_filter:
            
                    cv2.putText(color_image, f"Target: {active_filter.upper()}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # overlaying activation warning notices
                # if systems wait for input signals
                if current_state == 'WAIT_FOR_EMG':
            
                    cv2.putText(color_image, f"READY: Wrist Locked! Awaiting EMG...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Stitching telemetry frames together
                # to render parallel viewport frames
                output_window = np.hstack((color_image, depth_colormap))
                cv2.imshow("Semi-Autonomous Grasp Vision System", output_window)
                cv2.waitKey(1)
            
                continue

            # Running spatial bounding calculations
            # across active color image arrays
            results = model(color_image, stream=True, verbose=False)
            valid_candidates = []

            # Extracting spatial position details
            # within secure runtime locks
            with node.lock:
            
                current_wrist_pos = node.current_position
            
            is_rotated_right = current_wrist_pos > 2000

            # Iterating through network tracking loops
            # to filter candidate target metrics
            for result in results:
            
                for box in result.boxes:

                    # Dropping low assurance candidates
                    # using minimum confidence constraints
                    if float(box.conf[0]) < 0.4:
            
                        continue

                    # Comparing model category names
                    # against chosen target filters
                    label = model.names[int(box.cls[0])]
            
                    if label != active_filter:
            
                        continue

                    # Mapping border bounding coordinates
                    # to pinpoint image frame centers
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = np.clip((x1 + x2) // 2, 0, depth_image.shape[1] - 1)
                    cy = np.clip((y1 + y2) // 2, 0, depth_image.shape[0] - 1)

                    # Collecting neighborhood depth matrices
                    # around spatial center focal points
                    patch = depth_image[max(0, cy-2):min(depth_image.shape[0], cy+3), max(0, cx-2):min(depth_image.shape[1], cx+3)]
                    valid_depths = patch[patch > 0]
                    dist = np.median(valid_depths) * depth_scale if len(valid_depths) > 0 else -1.0

                    # Storing verified proximity targets
                    # inside local tracking lists
                    if 0.0 < dist <= 1.0:
            
                        valid_candidates.append({'box': (x1, y1, x2, y2, cx, cy), 'distance': dist, 'label': label})

            # Establishing baseline variable defaults
            # before evaluating candidate listings
            auto_left, auto_right, auto_center = False, False, False
            chosen_grasp, best_label, best_dist, best_box = 'cylindrical', "None", -1.0, None
            target_acquired = False

            # Isolating nearest target representations
            # to parse geometric structural profiles
            if len(valid_candidates) > 0:
            
                valid_candidates.sort(key=lambda item: item['distance'])
                closest = valid_candidates[0]
                best_box = closest['box']
                best_dist = closest['distance']
                best_label = closest['label']
                chosen_grasp = grasp_mapping.get(best_label, 'cylindrical')

                # Calculating structural aspect parameters
                # to assess object spatial alignment
                x1, y1, x2, y2, cx, cy = best_box
                p_w, p_h = x2 - x1, y2 - y1
                t_w, t_h = (p_h, p_w) if is_rotated_right else (p_w, p_h)

                # Shifting rotation output markers
                # based on dimension shape relationships
                if t_h > (1.3 * t_w):
            
                    auto_right = True
            
                else:
            
                    auto_center = True
                
                # Raising internal validation parameters
                # to complete target acquisition loops
                target_acquired = True 

                # Drawing tracking canvas borders
                # across visible output frame structures
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

            # Overlaying lookup missing warning notices
            # if target items disappear
            else:
            
                cv2.putText(color_image, f"SEARCHING FOR: {active_filter.upper()}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Updating shared control parameters
            # within secure runtime locks
            with node.lock:
            
                node.suggested_grasp = chosen_grasp
                node.tracked_object_label = best_label
                node.tracked_object_dist = best_dist
                node.moving_left = auto_left
                node.moving_right = auto_right
                node.moving_center = auto_center
                
                # Transitioning active state parameters
                # to maintain workflow step changes
                if target_acquired:
            
                    node.system_state = 'WAIT_FOR_EMG'
                    node.get_logger().info(f"Orientation locked! Moving wrist. Transitioning to WAIT_FOR_EMG State.")

            # Combining separate telemetry images
            # to present ongoing tracking video
            output_window = np.hstack((color_image, depth_colormap))
            cv2.imshow("Semi-Autonomous Grasp Vision System", output_window)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
            
                break
    finally:
        
        pipeline.stop()
        cv2.destroyAllWindows()


# intercepting muscular trigger messages
# from low-level muscle signal servers
def socket_receiver_thread(node):

    # Establishing endpoint network parameters
    # and mapping code translation lists
    host, port = '0.0.0.0', 5000
    commands = {'0': 'relax', '2': 'execute_grasp'}

    # Binding interface network structures
    # to host signal monitoring ports
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen()

        # Monitoring muscle signal lines
        # while operational communication links persist
        while rclpy.ok():
        
            conn, _ = s.accept()
        
            with conn:
        
                while True:
        
                    try:

                        # Reading incoming signal codes
                        # passing across open connection pipelines
                        raw_data = conn.recv(1024)
        
                        if not raw_data:
        
                            break 
                        
                        # Transcribing binary network inputs
                        # into sanitized string identifiers
                        data = raw_data.decode().strip()
                        
                        # Dropping unregistered command indices
                        # to avoid system execution faults
                        if data not in commands:
        
                            continue

                        # Inspecting system workflow states
                        # within secure runtime locks
                        with node.lock:
        
                            current_state = node.system_state

                        # Neutralizing historic confirm buffers
                        # if vocal resets change system configurations
                        if current_state in ['IDLE', 'GET_TARGET', 'TRACKING']:
        
                            node.consecutive_count = 0
                            node.last_received_id = None
        
                            continue

                        # Incrementing stability check counts
                        # to confirm structural command continuity
                        if data == node.last_received_id:
        
                            node.consecutive_count += 1
        
                        else:
        
                            node.last_received_id, node.consecutive_count = data, 1

                        # Rejecting transient command flickers
                        # if safety count filters are unfulfilled
                        if node.consecutive_count < node.required_confirmations:
        
                            continue

                        # Clearing checking buffer indexes
                        # and recording baseline transition points
                        node.consecutive_count = 0
                        cmd = commands[data]
                        start_time = time.perf_counter()

                        # Executing matching grip changes
                        # within secure runtime locks
                        with node.lock:

                            # Intercepting grasp trigger calls
                            # to lock active prosthetic grips
                            if cmd == 'execute_grasp' and node.system_state == 'WAIT_FOR_EMG':
        
                                node.system_state = 'GRASPED'
        
                                node.moving_left = False
                                node.moving_right = False
                                node.moving_center = False
                                
                                # Launching remote finger manipulation
                                # matching identified item parameters
                                active_grasp = node.suggested_grasp
                                node.get_logger().info(f"[EMG GESTURE 2] Executing grip: {active_grasp.upper()}")
                                node.send_grasp(active_grasp, start_time)

                            # Intercepting release trigger calls
                            # to open finger segments cleanly
                            elif cmd == 'relax' and node.system_state == 'GRASPED':
        
                                node.get_logger().info("[EMG GESTURE 0] Relax detected. Opening fingers...")
                                node.send_grasp('relax', start_time)
                                
                                # Preserving fixed orientation properties
                                # while waiting for reset instructions
                                node.system_state = 'RELEASED_WAITING'
        
                                node.moving_left = False
                                node.moving_right = False
                                node.moving_center = False  
        
                                node.get_logger().info("=== Hand opened. Holding orientation. Awaiting 'GO' voice command to home... ===")
                    
                    # Intercepting data transfer disruptions
                    # by printing localized warning flags
                    except Exception as e:
        
                        node.get_logger().warn(f"Socket parsing warning: {e}")
        
                        break


# Executing the main function
# to run the system controllers
def main():

    # Initializing framework client channels
    # to register software interaction threads
    rclpy.init()
    node = MiaPhysicalController()

    # Spawning parallel background threads
    # to isolate distinct operational tasks
    threading.Thread(target=run_voice_target_selection, args=(node,), daemon=True).start()
    threading.Thread(target=socket_receiver_thread, args=(node,), daemon=True).start()
    threading.Thread(target=node.wrist_movement_engine, daemon=True).start()
    threading.Thread(target=vision_processing_thread, args=(node,), daemon=True).start()

    # Managing clean hardware teardowns
    # during program termination scenarios
    def shutdown_sequence():
    
        node.emergency_stop()
    
        if hasattr(node, 'port_handler') and node.port_handler.is_open:
    
            node.port_handler.closePort()
    
        rclpy.shutdown()
        sys.exit(0)

    # Registering operating system callbacks
    # to intercept closing environment interrupts
    signal.signal(signal.SIGVERSION if sys.platform == 'win32' else signal.SIGTERM, lambda s, f: shutdown_sequence())
    
    # Sustaining framework polling updates
    # safely inside an error handling block
    try:
    
        while rclpy.ok():
    
            time.sleep(0.1)
    
    except KeyboardInterrupt:
    
        shutdown_sequence()


# Direct script execution
if __name__ == '__main__':

    # Invoking the main function
    main() 
