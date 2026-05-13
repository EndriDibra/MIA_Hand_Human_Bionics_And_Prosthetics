# Author: Endri Dibra 

# Project: Dynamixel XM540-W270 Solo Controller

# Description: Independent ROS 2 node to control wrist rotation via U2D2

# Importing core ROS 2 client library
import rclpy

# Importing Node for creating the controller
from rclpy.node import Node

# Importing everything from Dynamixel SDK for serial communication
from dynamixel_sdk import *


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
DEVICENAME = '/dev/ttyUSB1' 

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
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_OPERATING_MODE, 3)

        # Writing one byte to enable motor torque
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

        # Logging that the wrist is ready for rotation
        self.get_logger().info("Wrist Torque Enabled. Ready to rotate.")

    # Processing the movement to a specific pulse
    def move_to_pulse(self, pulse):

        # Clamping the pulse value within safe hardware limits
        pulse = max(0, min(4095, pulse)) 

        # Writing four bytes to send the goal position
        self.packet_handler.write4ByteTxRx(self.port_handler, DXL_ID, ADDR_GOAL_POSITION, pulse)

        # Logging the newly commanded position
        self.get_logger().info(f"Commanded Position: {pulse}")

    # Reading the current motor position
    def get_position(self):

        # Executing the read command for present position
        present_pos, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, DXL_ID, ADDR_PRESENT_POSITION)

        # Returning the obtained position value
        return present_pos

    # Handling the safe shutdown of the motor
    def shutdown(self):

        # Writing one byte to disable motor torque
        self.packet_handler.write1ByteTxRx(self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, 0)

        # Closing the serial port connection
        self.port_handler.closePort()

        # Logging the successful closure of the system
        self.get_logger().info("Torque disabled and Port closed.")

# Executing the main application logic
def main():

    # Starting the ROS 2 system
    rclpy.init()

    # Creating the wrist controller instance
    wrist = WristSoloController()

    # Defining the interactive user menu
    menu = """
    ===========================================
    DYNAMIXEL XM540 SOLO CONTROL
    ===========================================
    1: Rotate Left  (1600)
    2: Center       (2500)
    3: Rotate Right (3600)
    
    M: Manual Pulse Input (0-4095)
    G: Get Present Position
    0: Exit
    -------------------------------------------
    Selection: """

    # Monitoring user input within a loop
    try:

        # Continuing the loop until the user exits
        while True:

            # Capturing and converting input to uppercase
            choice = input(menu).upper()

            # Moving the wrist to the left position
            if choice == '1':

                # Sending the left pulse command
                wrist.move_to_pulse(1600)

            # Moving the wrist to the center position
            elif choice == '2':

                # Sending the center pulse command
                wrist.move_to_pulse(2500)

            # Moving the wrist to the right position
            elif choice == '3':

                # Sending the right pulse command
                wrist.move_to_pulse(3600)

            # Fetching the current motor position
            elif choice == 'G':

                # Obtaining position via the controller
                pos = wrist.get_position()

                # Printing the feedback to the console
                print(f"\n[FEEDBACK] Present Position: {pos}")

            # Entering manual pulse input mode
            elif choice == 'M':

                # Validating the user's numeric input
                try:

                    # Prompting the user for a pulse value
                    val = int(input("Enter pulse value (0-4095): "))

                    # Sending the custom pulse command
                    wrist.move_to_pulse(val)

                # Handling non-integer input errors
                except ValueError:

                    # Printing an error message for invalid numbers
                    print("Invalid input. Please enter a number.")

            # Breaking the loop to start shutdown
            elif choice == '0':

                # Exiting the while loop
                break

    # Catching manual interruptions from the keyboard
    except KeyboardInterrupt:

        # Passing through to finalize shutdown
        pass

    # Finalizing the system closure
    finally:

        # Disabling the motor and closing ports
        wrist.shutdown()

        # Shutting down the ROS 2 client library
        rclpy.shutdown()


# Checking for direct script execution
if __name__ == '__main__':

    # Invoking the main function
    main()