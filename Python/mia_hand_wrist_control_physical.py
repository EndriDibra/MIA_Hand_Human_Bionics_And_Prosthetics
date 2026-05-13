# Author: Endri Dibra 

# Project: Mia Hand Grasp Control and Integrated Wrist Rotation

# Description: This is a custom ROS 2 node providing an interface for Mia Hand grasps
# and controlling the Dynamixel XM540-W270 motor for wrist rotation

# Importing time for managing delays
import time

# Importing core ROS 2 client library
import rclpy

# Importing threading for background processing
import threading

# Importing Node for creating the controller
from rclpy.node import Node

# Importing standard empty service for safety
from std_srvs.srv import Empty

# Importing specific service types for Mia Hand
from mia_hand_msgs.srv import ExecuteGrasp

# Importing everything from Dynamixel SDK for motor communication
from dynamixel_sdk import * 


# Defining addresses for Dynamixel motor control
ADDR_TORQUE_ENABLE  = 64

# Defining address for sending goal position
ADDR_GOAL_POSITION = 116

# Defining address for reading current position
ADDR_PRESENT_POSITION = 132

# Defining address for selecting operating mode
ADDR_OPERATING_MODE = 11

# Specifying the protocol version as 2.0
PROTOCOL_VERSION = 2.0

# Assigning the ID for the wrist motor
DXL_ID = 1           

# Defining the baudrate for the U2D2 interface
BAUDRATE = 57600       

# Setting the serial port path for the wrist
DEVICENAME = '/dev/ttyUSB1' 

# Defining the main controller class
class MiaPhysicalController(Node):

    # Initializing the node and hardware interfaces
    def __init__(self):

        # Calling the parent Node constructor
        super().__init__('mia_physical_controller')

        # Adding a state variable to track if the hand is stopped
        self.is_stopped = False

        # Mapping grasp types to their respective service clients
        self.grasp_clients = {
            
            'cylindrical': self.create_client(ExecuteGrasp, '/mia_hand/grasps/cylindrical/execute'),
            'pinch':       self.create_client(ExecuteGrasp, '/mia_hand/grasps/pinch/execute'),
            'lateral':     self.create_client(ExecuteGrasp, '/mia_hand/grasps/lateral/execute'),
            'relax':       self.create_client(ExecuteGrasp, '/mia_hand/grasps/relax/execute')
        }

        # Creating the client for stopping motion
        self.stop_client = self.create_client(Empty, '/mia_hand/stop')

        # Creating the client for resuming motion
        self.play_client = self.create_client(Empty, '/mia_hand/play')

        # Initiating the Dynamixel port handler
        self.port_handler = PortHandler(DEVICENAME)

        # Initiating the Dynamixel packet handler
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Opening the serial port for the wrist motor
        if self.port_handler.openPort():

            # Logging the successful port connection
            self.get_logger().info(f"Succeeded in opening the port: {DEVICENAME}")

        # Handling the failure of opening the port
        else:

            # Reporting the port error to the console
            self.get_logger().error("Failed in opening the wrist port!")

        # Configuring the baudrate for the wrist motor
        if self.port_handler.setBaudRate(BAUDRATE):

            # Logging the successful baudrate setup
            self.get_logger().info(f"Succeeded in setting baudrate to: {BAUDRATE}")

        # Handling the failure of setting baudrate
        else:

            # Reporting the baudrate error to the console
            self.get_logger().error("Failed in setting the wrist baudrate!")

        # Writing the command to set Position Control Mode
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_OPERATING_MODE, 3)

        # Writing the command to enable wrist torque
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        # Printing initialization status for services
        self.get_logger().info("Waiting for Mia Hand services...")

        # Checking availability of all grasp services
        for name, client in self.grasp_clients.items():

            # Waiting for each service to respond
            if not client.wait_for_service(timeout_sec=2.0):

                # Reporting missing services in the log
                self.get_logger().error(f"Service {name} not available!")

        # Setting a default safety closing percentage
        self.close_percent = 50   

        # Setting a default safety force and speed percentage
        self.force_speed = 40     

        # Confirming interface readiness
        self.get_logger().info("The Integrated Mia Hand and Wrist Interface is Ready.")

    # Sending movement commands to the wrist motor
    def move_wrist(self, pulse):

        # Clamping the pulse value for mechanical safety
        pulse = max(0, min(4095, pulse)) 

        # Writing the goal position to the motor
        self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_GOAL_POSITION, pulse)

        # Logging the commanded pulse value
        self.get_logger().info(f"Moving wrist to pulse: {pulse}")

    # Processing and sending grasp requests
    def send_grasp(self, grasp_type):

        # Checking if the hand is currently locked in STOP mode
        if self.is_stopped:
            
            # Reporting the blocked status
            self.get_logger().error(f"BLOCKED: Hand is STOPPED. Press 'P' to Resume.")
            
            # Returning to ignore the request
            return

        # Validating the requested grasp type
        if grasp_type in self.grasp_clients:

            # Creating a new service request object
            request = ExecuteGrasp.Request()

            # Converting percentage to integer for the request
            request.close_percent = int(self.close_percent)

            # Converting speed to integer for the request
            request.spe_for_percent = int(self.force_speed)

            # Invoking the service asynchronously
            self.grasp_clients[grasp_type].call_async(request)

            # Logging the sent command details
            self.get_logger().info(f"Sent {grasp_type} command.")

    # Triggering the emergency stop command
    def emergency_stop(self):

        # Setting the state to True
        self.is_stopped = True

        # Sending an empty request to the stop service
        self.stop_client.call_async(Empty.Request())

        # Disabling torque on the wrist motor
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)

        # Warning the user that stop was triggered
        self.get_logger().warn("Emergency Stop Activated. Torque Disabled.")

    # Switching the hand back to play mode
    def resume_play(self):

        # Setting the state to False
        self.is_stopped = False

        # Sending an empty request to the play service
        self.play_client.call_async(Empty.Request())

        # Re-enabling torque on the wrist motor
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        # Informing the user that play mode is active
        self.get_logger().info("MIA Hand and Wrist set to PLAY mode.") 

# Executing the main function logic
def main():

    # Starting the ROS 2 system
    rclpy.init()

    # Creating an instance of the integrated controller
    hand = MiaPhysicalController()

    # Running the node in a separate background thread
    thread = threading.Thread(target=rclpy.spin, args=(hand,), daemon=True)

    # Commencing the background thread execution
    thread.start()

    # Formatting the user interface menu
    menu = """
    ===========================================
    MIA Hand & Wrist Integrated Interface
    ===========================================
    Status: {status} | Aperture: {c}% | Speed: {s}%
    -------------------------------------------
    1: Cylindrical    2: Pinch    3: Lateral    4: Relax
    
    L: Rotate Left    M: Center    R: Rotate Right
    
    C: Change Params  P: Play      S: STOP      0: Exit
    -------------------------------------------
    Selection: """

    # Running the interactive command loop
    try:

        # Monitoring user input continuously
        while True:

            # Updating the status visualizer
            status_text = "STOPPED" if hand.is_stopped else "READY"

            # Displaying menu and capturing uppercase input
            choice = input(menu.format(c=hand.close_percent, s=hand.force_speed, status=status_text)).upper()

            # Processing hand grasp commands
            if choice == '1': 
                
                hand.send_grasp('cylindrical')

            elif choice == '2': 
                
                hand.send_grasp('pinch')

            elif choice == '3': 
                
                hand.send_grasp('lateral')

            elif choice == '4':
                
                # Handling the temporary relaxation sequence
                temp_close = hand.close_percent
                hand.close_percent = 0 
                hand.send_grasp('relax')
                hand.close_percent = temp_close

            # Processing wrist rotation commands
            elif choice == 'L': 
                
                hand.move_wrist(1600)

            elif choice == 'M': 
                
                hand.move_wrist(2500)

            elif choice == 'R': 
                
                hand.move_wrist(3600)

            # Modifying hand parameters
            elif choice == 'C':

                # Catching input errors
                try:
                    
                    aperture = int(input("Enter Aperture % (0-100): "))
                    speed = int(input("Enter Speed % (0-99): "))
                    
                    hand.close_percent = max(0, min(100, aperture))
                    hand.force_speed = max(0, min(99, speed))
                
                except ValueError:
                    
                    print("Invalid numeric input.")

            # Triggering play mode
            elif choice == 'P':
                
                hand.resume_play()

            # Triggering stop mode
            elif choice == 'S':
                
                hand.emergency_stop()

            # Exiting the loop
            elif choice == '0':
                
                break

    # Catching manual exit commands
    except KeyboardInterrupt:
       
        pass

    # Ensuring safe shutdown of the hardware
    finally:

        # Invoking the stop procedure
        hand.emergency_stop()

        # Closing the serial port
        hand.port_handler.closePort()

        # Closing down the ROS 2 environment
        rclpy.shutdown()


# Checking for direct script execution
if __name__ == '__main__':

    # Starting the application
    main() 