# Author: Endri Dibra 

# Project: Mia Hand Grasp + Wrist Latency Benchmark System (EMG Stable Version)

# Description: Controlling physical Mia Hand grasps alongside 
# an external Dynamixel orientation wrist, managing communication 
# latency profiles, and tracking muscle signal noise filtration

# Importing network communication interface
# to handle remote input stream data packets
import socket

# Importing high resolution performance timers
# to measure system control loop latency marks
import time

# Importing robot operating system client library
# to spawn background node control execution architectures
import rclpy

# Importing multi-threading processing frameworks
# to run independent parallel task worker routines
import threading

# Importing operating system interrupt catch features
# to intercept external termination signal operations
import signal

# Importing framework system parameter features
# to manage direct software runtime termination exits
import sys

# Importing base node structures
# to implement ros2 execution objects
from rclpy.node import Node

# Importing universal empty service definitions
# to pass basic toggle messages to peripherals
from std_srvs.srv import Empty

# Importing custom grasp execute interface definitions
# to request specialized multi-joint posture movements
from mia_hand_msgs.srv import ExecuteGrasp

# Importing dynamixel smart actuator control development kit
# to manage low level serial communication protocol registers
from dynamixel_sdk import *


# Defining operational address mapping codes
# matching the dynamixel hardware configurations
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11

# Specifying connection protocol standard rules
# targeting active robot hardware motor architectures
PROTOCOL_VERSION = 2.0
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB1' 


# Implementing the primary robotic controller node
# to orchestrate hand and wrist tracking updates
class MiaPhysicalController(Node):

    # Constructing internal node parameters
    # to register peripheral communication interfaces
    def __init__(self):

        # Initializing the base instance name
        # inside the local client topology tree
        super().__init__('mia_physical_controller')

        # Instantiating a thread exclusion flag lock
        # to safeguard shared state variable manipulation updates
        self.lock = threading.Lock()

        # Tracking consecutive incoming command sequences
        # to filter accidental brief sensor fluctuations cleanly
        self.last_received_id = None
        self.consecutive_count = 0
        self.required_confirmations = 2
        self.motion_start_time = None

        # Instantiating the low level port interaction driver
        # to map direct access to physical serial connections
        self.port_handler = PortHandler(DEVICENAME)

        # Instantiating data packet protocol translation layers
        # to compile register messages for target hardware elements
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Validating communication channel availability properties
        # to verify peripheral link setups before execution steps
        if not self.port_handler.openPort():

            # Terminating initial setup steps
            # if physical wiring blocks disconnect
            self.get_logger().error("PORT OPEN FAILED")
           
            return

        # Synchronizing system communication data cycle rates
        # with internal motor firmware configuration registries
        if not self.port_handler.setBaudRate(BAUDRATE):

            # Halting setup procedures if matching
            # configuration rate operations fall out of bounds
            self.get_logger().error("BAUDRATE FAILED")
           
            return

        # Transmitting structural operating setup registers
        # to place target hardware components into position mode
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_OPERATING_MODE, 3)
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        # Restricting motor positioning travel boundaries
        # to avoid internal physical linkage collision stress
        self.STEP_SIZE = 200
        self.UPDATE_DELAY = 0.03
        self.MIN_POSITION = 700
        self.MAX_POSITION = 2500
        self.CENTER_POSITION = 1600

        # Syncing local position status tracking
        # to match current motor internal encoder steps
        self.current_position = self.get_position()

        # Resetting directional transition state properties
        # to configure balanced baseline start conditions
        self.moving_left = False
        self.moving_right = False
        self.moving_center = False
        self.is_stopped = False

        # Binding standalone individual interface links
        # across available target finger closure configurations
        self.grasp_clients = {
            
            'cylindrical': self.create_client(ExecuteGrasp, '/mia_hand/grasps/cylindrical/execute'),
            'pinch': self.create_client(ExecuteGrasp, '/mia_hand/grasps/pinch/execute'),
            'lateral': self.create_client(ExecuteGrasp, '/mia_hand/grasps/lateral/execute'),
            'relax': self.create_client(ExecuteGrasp, '/mia_hand/grasps/relax/execute')
        }
       
        self.stop_client = self.create_client(Empty, '/mia_hand/stop')
        self.play_client = self.create_client(Empty, '/mia_hand/play')

        self.get_logger().info("Mia Controller Ready (EMG Stable Mode)")

    # Recording command response runtime delays
    # to evaluate total network loop processing times
    def log_latency(self, label, start):

        # Computing total elapsed millisecond intervals
        # relative to original execution kickoff marks
        t = (time.perf_counter() - start) * 1000
      
        self.get_logger().info(f"{label} LATENCY {t:.2f} ms")

    # Transmitting target position coordinate values
    # directly down into hardware encoder storage registries
    def move_wrist(self, pulse):

        # Enforcing software hard travel boundary constraints
        # to protect custom physical hardware limits safely
        pulse = max(self.MIN_POSITION, min(self.MAX_POSITION, pulse))

        # Committing the clamped target location value
        # into the active hardware position control register
        self.packet_handler.write4ByteTxRx(
      
            self.port_handler,
            DXL_ID,
            ADDR_GOAL_POSITION,
            pulse
        )

    # Querying the active hardware register values
    # to retrieve current raw position telemetry feedback
    def get_position(self):

        # Fetching current orientation encoder data units
        # directly from the internal peripheral microcontroller
        pos, _, _ = self.packet_handler.read4ByteTxRx(
      
            self.port_handler,
            DXL_ID,
            ADDR_PRESENT_POSITION
        )
        return pos

    # Running continuous trajectory stepping updates
    # to manage incremental wrist transition paths smoothly
    def wrist_movement_engine(self):

        # Maintaining active location calculation steps
        # while system node execution loops remain healthy
        while rclpy.ok():

            # Postponing tracking recalculation loops
            # if safety freeze flags block operation
            if self.is_stopped:
       
                time.sleep(self.UPDATE_DELAY)
       
                continue

            # Preserving a blank temporary movement target holder
            # to verify modification updates before writing to hardware
            target_to_send = None

            # Isolating block access tracking states
            # securely inside synchronous thread block zones
            with self.lock:

                # Checking active negative step targets
                # to execute counter clockwise rotation paths
                if self.moving_left:

                    # Decrementing ongoing target location entries
                    # if current steps remain above software bounds
                    if self.current_position > self.MIN_POSITION:
               
                        self.current_position -= self.STEP_SIZE
               
                        target_to_send = self.current_position

                    # Terminating ongoing rotation tracking adjustments
                    # once hardware reaches terminal left stop positions
                    else:
               
                        self.moving_left = False
               
                        if self.motion_start_time:
               
                            self.log_latency("LEFT DONE", self.motion_start_time)
                            self.motion_start_time = None

                # Checking active positive step targets
                # to execute clockwise rotation paths
                elif self.moving_right:

                    # Incrementing ongoing target location entries
                    # if current steps remain below software bounds
                    if self.current_position < self.MAX_POSITION:
               
                        self.current_position += self.STEP_SIZE
               
                        target_to_send = self.current_position

                    # Terminating ongoing rotation tracking adjustments
                    # once hardware reaches terminal right stop positions
                    else:
               
                        self.moving_right = False
               
                        if self.motion_start_time:
               
                            self.log_latency("RIGHT DONE", self.motion_start_time)
                            self.motion_start_time = None

                # Checking home location return loops
                # to center the hardware orientation smoothly
                elif self.moving_center:

                    # Setting the absolute destination coordinate
                    # representing the physical assembly alignment center
                    target = self.CENTER_POSITION

                    # Recording structural movement initiation stamps
                    # if initial tracking properties arrive completely blank
                    if self.motion_start_time is None:
               
                        self.motion_start_time = time.perf_counter()

                    # Estimating remaining angular encoder step distances
                    # separating current coordinates from home positions
                    diff = self.current_position - target

                    # Snapping location values directly onto goals
                    # if remaining steps fall under standard increment limits
                    if abs(diff) <= self.STEP_SIZE:
                        
                        self.current_position = target
                        target_to_send = self.current_position
                        self.moving_center = False

                        self.log_latency("CENTER DONE", self.motion_start_time)
                        self.motion_start_time = None

                    # Stepping backward toward zero points
                    # if tracking coordinates overshoot target zones
                    elif diff > 0:
                    
                        self.current_position -= self.STEP_SIZE
                    
                        target_to_send = self.current_position

                    # Stepping forward toward zero points
                    # if tracking coordinates undershoot target zones
                    else:
                  
                        self.current_position += self.STEP_SIZE
                  
                        target_to_send = self.current_position

            # Transporting updated target step values
            # directly into active motor management functions
            if target_to_send is not None:
              
                self.move_wrist(target_to_send)

            # Pausing loop execution cycles briefly
            # to regulate fixed update rate distributions
            time.sleep(self.UPDATE_DELAY)

    # Launching asynchronous finger target requests
    # to control internal prosthetic motor closure levels
    def send_grasp(self, grasp_type, start_time):

        # Rejecting action initiation requests completely
        # if safety flags trigger or target names mismatch
        if self.is_stopped or grasp_type not in self.grasp_clients:
          
            return

        # Mapping finger endpoint target scales
        # for individual mechanical closure layouts
        grasp_apertures = {
          
            'cylindrical': 85,
            'pinch': 85,
            'lateral': 55,
            'relax': 0
        }

        # Compiling structural profile request definitions
        # to define speed and displacement percentage ratios
        req = ExecuteGrasp.Request()
        req.close_percent = grasp_apertures.get(grasp_type, 50)
        req.spe_for_percent = 30

        self.get_logger().info(f"STARTING PHYSICAL {grasp_type.upper()}...")
        
        # Dispatching requests across the non-blocking client link
        # to pass joint operations to background communication loops
        future = self.grasp_clients[grasp_type].call_async(req)

        # Attaching completion tracking response loops
        # to measure total hardware execution latency intervals
        def callback(fut):
        
            try:
        
                # Retrieving peripheral execution results
                # to confirm remote service process fulfillment
                response = fut.result()
        
                self.log_latency(f"PHYSICAL {grasp_type.upper()} COMPLETE", start_time)
        
            except Exception as e:
        
                self.get_logger().error(f"Grasp failed: {str(e)}")

        # Registering completion monitoring tracking logic
        # into the active background service handler profile
        future.add_done_callback(callback)

    # Halting downstream component trajectories immediately
    # to lock active hardware units against unexpected operation
    def emergency_stop(self):

        # Forcing background motion control flags down
        # within isolated secure thread locking environments
        with self.lock:
        
            self.is_stopped = True
            self.moving_left = False
            self.moving_right = False
            self.moving_center = False

        # Snapping active motor destination targets
        # directly to match current stationary positions
        pos = self.get_position()
        self.move_wrist(pos)

        self.get_logger().warn(f"STOP AT {pos}")


# Initializing incoming data packet listener interfaces
# inside independent background socket processing threads
def socket_receiver_thread(node):

    # Standardizing incoming binding addresses
    # across active interface network gateway links
    host = '0.0.0.0'
    port = 5000

    # Indexing unique string character keys
    # to map incoming inputs to target commands
    commands = {
    
        '0': 'relax',
        '1': 'center',
        '2': 'lateral',
        '3': 'cylindrical',
        '4': 'pinch',
        '5': 'left',
        '6': 'right'
    }

    # Configuring system networking socket layers
    # inside an automated structural cleanup block
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        
        # Adjusting channel reusable configuration options
        # to allow rapid connection resetting across ports
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen()

        node.get_logger().info("Socket Ready (EMG Filtering Active)")

        # Reviewing input incoming channel connection loops
        # while underlying system structures stay operational
        while rclpy.ok():

            # Blocking loop execution parameters until
            # an external remote sender initiates contact
            conn, _ = s.accept()

            # Managing continuous stream communication data
            # safely inside active remote network session paths
            with conn:

                # Extraction processing loop variables
                # running throughout active socket stream cycles
                while True:

                    # Extracting raw data packet segments
                    # and stripping away trailing newline items
                    data = conn.recv(1024).decode().strip()

                    # Breaking active session loop steps
                    # if empty incoming drop signals trigger
                    if not data:
                 
                        break

                    # Dropping unrecognized input stream data
                    # if codes fail to match standard commands
                    if data not in commands:
                 
                        continue

                    # Filtering transient background sensor ripples
                    # by checking duplicate sequential value codes
                    if data == node.last_received_id:
                 
                        node.consecutive_count += 1
                 
                    else:
                 
                        node.last_received_id = data
                 
                        node.consecutive_count = 1

                    # Skipping onward movement logic steps
                    # until confirmation counters clear noise limits
                    if node.consecutive_count < node.required_confirmations:
                 
                        continue

                    # Resetting filter tracking sequence marks
                    # once clean intended inputs clear validation steps
                    node.consecutive_count = 0

                    # Isolating matching action tracking names
                    # and logging high resolution performance marks
                    cmd = commands[data]
                    start_time = time.perf_counter()

                    # Assigning target movement routing flags
                    # within thread safe variable isolation spaces
                    with node.lock:

                        # Triggering counter clockwise rotation parameters
                        if cmd == 'left':
                           
                            node.motion_start_time = start_time
                           
                            node.moving_left = True
                            node.moving_right = False
                            node.moving_center = False

                        # Triggering clockwise rotation parameters
                        elif cmd == 'right':
                           
                            node.motion_start_time = start_time
                           
                            node.moving_right = True
                            node.moving_left = False
                            node.moving_center = False

                        # Triggering centering realignment tracking properties
                        elif cmd == 'center':
                           
                            node.motion_start_time = start_time
                           
                            node.moving_center = True
                            node.moving_left = False
                            node.moving_right = False

                        # Dropping joint pressure metrics down
                        # to switch fingers into compliant states
                        elif cmd == 'relax':
                           
                            node.moving_left = False
                            node.moving_right = False
                            node.moving_center = False
                           
                            node.send_grasp('relax', start_time)

                        # Dispatched targeted grip posture patterns
                        # directly to background service handler client nodes
                        else:
                           
                            node.send_grasp(cmd, start_time)


# Executing the main initialization setups
# to run the primary control loops
def main():

    # Initializing underlying robot communication environments
    # to enable internal messaging structures across nodes
    rclpy.init()
    
    # Constructing core physical interaction controller nodes
    # to handle peripheral device management loop parameters
    node = MiaPhysicalController()

    # Launching parallel streaming connection network threads
    # to monitor incoming instructions without blocking nodes
    threading.Thread(target=socket_receiver_thread, args=(node,), daemon=True).start()
    
    # Launching parallel trajectory calculation loops
    # to maintain steady update rate distributions for motors
    threading.Thread(target=node.wrist_movement_engine, daemon=True).start()

    # Creating structural system interrupt handler tasks
    # to catch termination events and release equipment cleanly
    def handler(sig, frame):
        
        node.emergency_stop()
        node.port_handler.closePort()
        rclpy.shutdown()
        sys.exit(0)

    # Intercepting standard shutdown signals
    # to route control flow into cleanup sequences
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    # Locking main execution thread properties down
    # to preserve ongoing messaging callbacks across nodes
    rclpy.spin(node)


# Direct script execution
if __name__ == '__main__':

    # Invoking the main function
    main() 
