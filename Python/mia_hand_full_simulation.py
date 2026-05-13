# Author: Endri Dibra 

# Project: Mia Hand Grasp Control and Simulation Interface

# Description: This is a custom ROS 2 node that provides a command-line interface
# for controlling the Mia Hand to perform 3 different grasping types in a simulation environment

# Importing the time module for
# managing delays and timeouts
import time

# Importing the socket library
# to enable network communication
import socket

# Importing the core ROS 2 client library  
import rclpy

# Importing the threading module
# for running the GUI and ROS concurrently
import threading

# Importing the main tkinter module
# for the graphical user interface
import tkinter as tk

# Importing the tkinter themed
# widgets for a modern UI look
from tkinter import ttk

# Importing the Node class
# for creating ROS 2 nodes
from rclpy.node import Node

# Importing the joint state
# message type for receiving telemetry
from sensor_msgs.msg import JointState

# Importing the multi-array message
# type for sending joint commands
from std_msgs.msg import Float64MultiArray


# Defining the main research class
# inheriting from the ROS 2 Node
class MiaHandResearcher(Node):

    # Initializing the node and setting up
    # publishers and subscribers
    def __init__(self):

        # Naming the ROS 2 node 'mia_hand_researcher'
        super().__init__('mia_hand_researcher')

        # Creating a state variable to lock/unlock simulation movement
        self.is_stopped = False

        # Creating the publisher for the index finger controller
        self.index_pub = self.create_publisher(Float64MultiArray, '/index_pos_ff_controller/commands', 10)

        # Creating the publisher for the thumb flexion controller
        self.thumb_pub = self.create_publisher(Float64MultiArray, '/thumb_pos_ff_controller/commands', 10)

        # Creating the publisher for the middle-ring-little finger group
        self.mrl_pub   = self.create_publisher(Float64MultiArray, '/mrl_pos_ff_controller/commands', 10)

        # Subscribing to the joint states topic for real-time MuJoCo feedback
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Initializing an empty dictionary for storing incoming joint data
        self.latest_joint_data = {}

        # Initializing the target positions for the three motor groups
        self.targets = {"index": 0.0, "thumb": 0.0, "mrl": 0.0}

        # Initializing the current command state to track interpolated movement
        self.current_cmds = {"index": 0.0, "thumb": 0.0, "mrl": 0.0}

        # Setting the default movement speed in radians per second
        self.speed = 0.5

        # Defining the maximum allowed speed for hardware and simulation safety
        self.MAX_SPEED = 1.0

        # Defining the 100Hz timer period in seconds
        self.timer_period = 0.01

        # Creating a recurring timer to execute the control loop every 10ms
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Logging the successful initialization of the 100Hz interface
        self.get_logger().info(f'Interface Ready at 100Hz. Safety Cap: {self.MAX_SPEED} rad/s')

    # Defining the callback for processing incoming joint state messages
    def joint_callback(self, msg):

        # Iterating through the joint names, positions, and velocities provided by MuJoCo
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):

            # Updating the dictionary with the latest position and velocity data
            self.latest_joint_data[name] = {"pos": pos, "vel": vel}

    # Defining the high-frequency control loop for smooth motion
    def control_loop(self):

        # If stopped, current_cmds are not updated,
        # so that to effectively freeze/stop the hand
        if self.is_stopped:
            return

        # Calculating the maximum movement increment for the current time step
        step_size = self.speed * self.timer_period

        # Processing each motor group to move towards its target
        for key in ["index", "thumb", "mrl"]:

            # Calculating the difference between the target and current command
            diff = self.targets[key] - self.current_cmds[key]

            # Checking if the difference is larger than the allowed step size
            if abs(diff) > step_size:

                # Incrementing or decrementing the current command by the step size
                self.current_cmds[key] += step_size if diff > 0 else -step_size

            else:

                # Setting the command exactly to the target if close enough
                self.current_cmds[key] = self.targets[key]

        # Publishing the updated index finger command
        self.index_pub.publish(Float64MultiArray(data=[self.current_cmds["index"]]))

        # Publishing the updated thumb flexion command
        self.thumb_pub.publish(Float64MultiArray(data=[self.current_cmds["thumb"]]))

        # Publishing the updated middle-ring-little group command
        self.mrl_pub.publish(Float64MultiArray(data=[self.current_cmds["mrl"]]))

    # Defining a helper function to set all motor targets simultaneously
    def set_target(self, i, t, m):

        # Preventing to set new targets if emergency stop is active
        if self.is_stopped:
            
            self.get_logger().error("BLOCKED: Cannot change target while SIMULATION is STOPPED.")
            
            return 

        # Converting and storing the index target
        self.targets["index"] = float(i)

        # Converting and storing the thumb target
        self.targets["thumb"] = float(t)

        # Converting and storing the MRL group target
        self.targets["mrl"]   = float(m)

# Defining the function to manage the graphical telemetry dashboard
def run_gui_dashboard(node):

    # Creating the main tkinter window instance
    root = tk.Tk()

    # Setting the title of the telemetry window
    root.title("Mia Hand: Position & Velocity (100Hz)")

    # Defining the initial dimensions of the window
    root.geometry("450x420")

    # Forcing the window to stay on top of other applications
    root.attributes('-topmost', True)

    # Creating the main header label for the dashboard
    ttk.Label(root, text="LIVE TELEMETRY (rad/s)", font=("Arial", 12, "bold")).pack(pady=10)

    # Creating a frame for displaying the currently commanded speed
    speed_frame = ttk.Frame(root)

    # Positioning the speed frame with padding
    speed_frame.pack(pady=5)

    # Adding a static label for the speed description
    ttk.Label(speed_frame, text="Commanded Speed:", font=("Arial", 10)).pack(side="left")

    # Creating a dynamic label to show the live speed value
    speed_val_lbl = ttk.Label(speed_frame, text="0.50", font=("Arial", 10, "bold"), foreground="blue")

    # Positioning the live speed value with horizontal padding
    speed_val_lbl.pack(side="left", padx=5)

    # Mapping the technical joint names to user-friendly display names
    FINGER_MAP = {
       
        "j_index_fle": "INDEX FINGER",
        "j_thumb_fle": "THUMB FLEXION",
        "j_mrl_fle":   "M/R/L GROUP",
        "j_thumb_opp": "THUMB OPPOSIT."
    }

    # Initializing a dictionary to hold the label objects for updates
    labels = {}

    # Iterating through the finger map to build the dashboard rows
    for tech_name, full_name in FINGER_MAP.items():

        # Creating a horizontal frame for each joint row
        frame = ttk.Frame(root)

        # Positioning the frame to fill the horizontal space
        frame.pack(fill="x", padx=25, pady=6)

        # Determining the label color based on the joint type
        name_color = "darkblue" if "opp" in tech_name else "black"

        # Adding the finger name label to the current row
        ttk.Label(frame, text=f"{full_name}:", font=("Arial", 10, "bold"), width=18, foreground=name_color).pack(side="left")

        # Creating the value label for displaying position and velocity
        val_lbl = ttk.Label(frame, text="P: 0.000 | V: 0.000", font=("Courier", 11))

        # Positioning the value label to the right side of the row
        val_lbl.pack(side="right")

        # Storing the label object in the dictionary using the technical name
        labels[tech_name] = val_lbl

    # Defining the internal loop for refreshing the GUI data
    def update_loop():

        # Refreshing the speed label with the latest node value
        speed_val_lbl.config(text=f"{node.speed:.2f}")

        # Checking if any joint data has been received yet
        if node.latest_joint_data:

            # Iterating through the joints tracked by the labels
            for tech_name, lbl in labels.items():

                # Checking if the specific joint exists in the latest data
                if tech_name in node.latest_joint_data:

                    # Retrieving the position and velocity for the joint
                    data = node.latest_joint_data[tech_name]

                    pos = data['pos']

                    vel = data['vel']

                    # Choosing a green color if the joint is flexed and red if open
                    text_color = "#27ae60" if abs(pos) > 0.05 else "#c0392b"

                    # Updating the label text with formatted position and velocity
                    lbl.config(
                        
                        text=f"P: {pos:>6.3f} | V: {vel:>6.3f}", 
                        foreground=text_color
                    )

        # Scheduling the next GUI update in 40 milliseconds
        root.after(40, update_loop)

    # Triggering the first iteration of the update loop
    update_loop()

    # Starting the main tkinter event loop
    root.mainloop()

# Defining the socket receiver thread for handling incoming data
def socket_receiver_thread(node): 
    
    # Setting up the server address to listen inside the container
    host = '0.0.0.0'

    # Defining the communication port
    port = 5000
    
    # Creating and managing the TCP/IP socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    
        # Binding the socket to the host and port
        s.bind((host, port))
       
        # Listening for incoming client connections
        s.listen()
       
        # Logging the server status to the ROS 2 node
        node.get_logger().info(f"Socket Server listening on port {port}...")

        # Maintaining the loop while the ROS 2 context is active
        while rclpy.ok():
    
            # Accepting a new connection from a client
            connection, address = s.accept()
    
            # Managing the established client connection
            with connection:
    
                # Processing incoming data packets continuously
                while True:
    
                    # Receiving and decoding the message string
                    data = connection.recv(1024).decode('utf-8')
    
                    # Breaking the loop if no data is received
                    if not data:
                        
                        break
                    
                    # Stripping whitespace from the received gesture identifier
                    gesture_id = data.strip()
                    
                    # Logging the received EMG gesture ID
                    node.get_logger().info(f"EMG Gesture Received: {gesture_id}")

                    # Evaluating the gesture ID for hand control actions
                    if gesture_id == '0':
    
                        # Setting the hand to a rest or open position
                        node.set_target(0.0, 0.0, 0.0)
    
                    # Handling the lateral grasp command
                    elif gesture_id == '2':
    
                        # Updating the target for lateral gripping
                        node.set_target(0.4, 0.8, 0.4)
    
                    # Handling the full grasp command
                    elif gesture_id == '3':
    
                        # Updating the target for full hand closure
                        node.set_target(0.8, 0.8, 0.8)
    
                    # Handling the pinch grasp command
                    elif gesture_id == '4':
    
                        # Updating the target for precise pinching
                        node.set_target(0.7, 0.7, 0.0)

# Establishing the primary entry point for the script
def main():
    
    # Initializing the ROS 2 communication layer
    rclpy.init()
    
    # Creating an instance of the hand researcher interface
    hand = MiaHandResearcher()

    # Starting a background thread for ROS 2 callback processing
    threading.Thread(target=rclpy.spin, args=(hand,), daemon=True).start()
    
    # Launching a dedicated thread for the telemetry visualization
    threading.Thread(target=run_gui_dashboard, args=(hand,), daemon=True).start()
    
    # Initiating the socket listener thread for remote gesture input
    threading.Thread(target=socket_receiver_thread, args=(hand,), daemon=True).start()

    # Managing the main application lifecycle
    try:
    
        # Printing the system readiness message
        print("System Active. Waiting for EMG Gestures from Windows...")
    
        # Keeping the main execution thread alive
        while rclpy.ok():
    
            # Pausing execution briefly to manage CPU usage
            time.sleep(1)
    
    # Handling user termination requests
    except KeyboardInterrupt:
    
        pass
    
    # Performing cleanup operations before exiting
    finally:
    
        # Returning the hand to a neutral position
        hand.set_target(0.0, 0.0, 0.0)
        
        # Allowing time for the final command to transmit
        time.sleep(0.5)
        
        # Shutting down the ROS 2 system
        rclpy.shutdown()


# Verifying the script execution context
if __name__ == '__main__':
    
    # Running the main function
    main() 