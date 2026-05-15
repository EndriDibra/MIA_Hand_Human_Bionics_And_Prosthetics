# Author: Endri Dibra 

# Project: Dynamixel XM540-W270 Solo Controller

# Description: Continuous wrist control for Mia Hand wrist rotation

# Importing core ROS 2 client library
import rclpy

# Importing Node for creating the controller
from rclpy.node import Node

# Importing everything from Dynamixel SDK for serial communication
from dynamixel_sdk import *

# Importing threading for continuous movement
import threading

# Importing time for movement timing
import time


# Setting address for torque enabling
ADDR_TORQUE_ENABLE = 64

# Setting address for sending goal position
ADDR_GOAL_POSITION = 116

# Setting address for reading present position
ADDR_PRESENT_POSITION = 132

# Setting address for defining operating mode
ADDR_OPERATING_MODE = 11

# Identifying protocol version as 2.0
PROTOCOL_VERSION = 2.0

# Assigning the unique ID for the motor
DXL_ID = 1

# Defining the communication speed
BAUDRATE = 57600

# Specifying the serial port device path
DEVICENAME = '/dev/ttyUSB0'


# Declaring the wrist controller class inheriting from Node
class WristSoloController(Node):

    # Initializing the class and motor communication
    def __init__(self):

        # Constructing the ROS 2 node
        super().__init__('wrist_solo_controller')

        # Initializing the port handler for serial communication
        self.port_handler = PortHandler(DEVICENAME)

        # Initializing the packet handler for Protocol 2.0
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Checking if opening the serial port succeeds
        if self.port_handler.openPort():

            # Logging the successful port opening
            self.get_logger().info(f"Succeeded to open the port: {DEVICENAME}")

        # Handling the failure of port opening
        else:

            # Logging the error for failed port access
            self.get_logger().error("Failed to open the port!")

            # Returning to prevent further execution
            return

        # Checking if setting the baudrate succeeds
        if self.port_handler.setBaudRate(BAUDRATE):

            # Logging the successful baudrate change
            self.get_logger().info(f"Succeeded to change the baudrate to: {BAUDRATE}")

        # Handling the failure of baudrate setting
        else:

            # Logging the error for failed baudrate configuration
            self.get_logger().error("Failed to change the baudrate!")

            # Returning to prevent further execution
            return

        # Writing one byte to set mode to Position Control
        self.packet_handler.write1ByteTxRx(
            
            self.port_handler,
            DXL_ID,
            ADDR_OPERATING_MODE,
            3
        )

        # Writing one byte to enable motor torque
        self.packet_handler.write1ByteTxRx(
           
            self.port_handler,
            DXL_ID,
            ADDR_TORQUE_ENABLE,
            1
        )

        # Logging that the wrist is ready for rotation
        self.get_logger().info("Wrist Torque Enabled. Ready to rotate.")

    # Processing the movement to a specific pulse
    def move_to_pulse(self, pulse):

        # Clamping the pulse value within safe hardware limits
        pulse = max(0, min(4095, pulse))

        # Writing four bytes to send the goal position
        self.packet_handler.write4ByteTxRx(
           
            self.port_handler,
            DXL_ID,
            ADDR_GOAL_POSITION,
            pulse
        )

    # Reading the current motor position
    def get_position(self):

        # Executing the read command for present position
        present_pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            
            self.port_handler,
            DXL_ID,
            ADDR_PRESENT_POSITION)

        # Returning the obtained position value
        return present_pos

    # Handling the safe shutdown of the motor
    def shutdown(self):

        # Writing one byte to disable motor torque
        self.packet_handler.write1ByteTxRx(
         
            self.port_handler,
            DXL_ID,
            ADDR_TORQUE_ENABLE,
            0
        )

        # Closing the serial port connection
        self.port_handler.closePort()

        # Logging the successful closure of the system
        self.get_logger().info("Torque disabled and Port closed.")


# Executing the main application logic
def main():

    # Starting ROS 2 system
    rclpy.init()

    # Creating controller instance for wrist control
    wrist = WristSoloController()

    # Reading the initial wrist position from the motor
    current_position = wrist.get_position()

    # Movement settings defining step size and speed
    # amount of movement per cycle
    STEP_SIZE = 200

    # time delay between updates (smoother = lower)     
    UPDATE_DELAY = 0.03

    # Defining safe operating limits for the wrist
    MIN_POSITION = 1600
    MAX_POSITION = 3600

    # Movement state variables controlling direction modes
    moving_left = False
    moving_right = False
    moving_center = False

    # Background loop responsible for continuous motor updates
    def movement_loop():

        # Allowing inner function to modify outer variables
        nonlocal current_position
        nonlocal moving_left
        nonlocal moving_right
        nonlocal moving_center

        # Infinite loop for continuous control execution
        while True:

            # Continuous LEFT movement mode
            if moving_left:

                # Ensuring to stay within safe left limit
                if current_position > MIN_POSITION:

                    # Decreasing position step by step
                    current_position -= STEP_SIZE

                    # Sending the updated position to motor
                    wrist.move_to_pulse(current_position)

                else:

                    # Stopping movement if limit is reached
                    moving_left = False

                    print("\nLEFT LIMIT REACHED")

            # Continuous RIGHT movement mode
            elif moving_right:

                # Ensuring to stay within safe right limit
                if current_position < MAX_POSITION:

                    # Increasing position step by step
                    current_position += STEP_SIZE

                    # Sending the updated position to motor
                    wrist.move_to_pulse(current_position)

                else:

                    # Stopping movement if limit is reached
                    moving_right = False

                    print("\nRIGHT LIMIT REACHED")

            # Continuous CENTERING movement mode
            elif moving_center:

                # Computing distance from center position (2500)
                diff = current_position - 2500

                # If close enough to center, snapping to exact value
                if abs(diff) <= STEP_SIZE:

                    current_position = 2500

                    wrist.move_to_pulse(current_position)

                    moving_center = False

                    print("\nCENTER REACHED")

                # If position is above center, moving downward
                elif diff > 0:

                    current_position -= STEP_SIZE

                    wrist.move_to_pulse(current_position)

                # If position is below center, moving upward
                else:

                    current_position += STEP_SIZE

                    wrist.move_to_pulse(current_position)

            # Delaying to control update rate and smoothing motion
            time.sleep(UPDATE_DELAY)

    # Creating a separate thread for continuous motor updates
    movement_thread = threading.Thread(
       
        target=movement_loop,
        daemon=True
    )

    # Starting the movement thread
    movement_thread.start()

    # User input menu for manual control
    menu = """
    ===========================================
    CONTINUOUS WRIST CONTROL
    ===========================================

    1 : Start Moving Left
    2 : Move to Center 
    3 : Start Moving Right
    0 : Stop Movement

    G : Get Current Position
    Q : Quit Program

    -------------------------------------------
    Selection: """

    # Main loop handling user commands
    try:

        # Infinite loop waiting for user input
        while True:

            # Reading and normalizing user input
            choice = input(menu).upper()

            # Activating LEFT movement mode
            if choice == '1':

                moving_left, moving_right, moving_center = True, False, False

                print("Moving LEFT...")

            # Activating CENTER movement mode
            elif choice == '2':

                moving_left, moving_right, moving_center = False, False, True

                print("Moving to CENTER...")

            # Activating RIGHT movement mode
            elif choice == '3':

                moving_left, moving_right, moving_center = False, True, False

                print("Moving RIGHT...")

            # Stopping all movement immediately
            elif choice == '0':

                moving_left, moving_right, moving_center = False, False, False

                print("Movement STOPPED.")

            # Reading and displaying current motor position
            elif choice == 'G':

                pos = wrist.get_position()

                print(f"Current Position: {pos}")

            # Exiting program safely
            elif choice == 'Q':

                break

            # Handling invalid input
            else:

                print("Invalid Selection.")

    # Handling keyboard interruption (CTRL+C)
    except KeyboardInterrupt:

        pass

    # Safe shutdown procedure
    finally:

        # Ensuring all motion is stopped
        moving_left = False
        moving_right = False
        moving_center = False

        # Safely disabling motor and closing communication
        wrist.shutdown()

        # Shutdown ROS 2 system cleanly
        rclpy.shutdown()


# Checking if script is executed directly
if __name__ == '__main__':

    # Launching main control program
    main() 
