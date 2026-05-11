# Author: Endri Dibra 

# Project: Mia Hand Grasp Control and Physical Interface

# Description: This is a custom ROS 2 node that provides a command-line interface
# for controlling the Mia Hand to perform 4 different grasping types


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


# Defining the main controller class
class MiaPhysicalController(Node):

    # Initializing the node and service clients
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

        # Printing initialization status to the console
        self.get_logger().info("Waiting for Mia Hand services...")

        # Checking availability of all grasp services
        for name, client in self.grasp_clients.items():

            # Waiting for each service to respond
            if not client.wait_for_service(timeout_sec=2.0):

                # Reporting missing services in the log
                self.get_logger().error(f"Service {name} not available! Check if the driver is running.")

        # Setting a default safety closing percentage
        self.close_percent = 50   

        # Setting a default safety force and speed percentage
        self.force_speed = 40     

        # Confirming interface readiness
        self.get_logger().info("The Physical Mia Hand Interface is Ready.")

    # Processing and sending grasp requests
    def send_grasp(self, grasp_type):

        # Checking if the hand is currently locked in STOP mode
        if self.is_stopped:
            
            self.get_logger().error(f"BLOCKED: Cannot send {grasp_type} while hand is STOPPED. Press 'P' to Resume.")
            
            return

        # Validating the requested grasp type
        if grasp_type not in self.grasp_clients:

            # Exiting if the type is unknown
            return

        # Creating a new service request object
        request = ExecuteGrasp.Request()

        # Converting percentage to integer for the request
        request.close_percent = int(self.close_percent)

        # Converting speed to integer for the request
        request.spe_for_percent = int(self.force_speed)

        # Invoking the service asynchronously
        self.grasp_clients[grasp_type].call_async(request)

        # Logging the sent command details
        self.get_logger().info(f"Sent {grasp_type}: Aperture={self.close_percent}%, Speed={self.force_speed}%")

    # Triggering the emergency stop command
    def emergency_stop(self):

        # Setting the state to True
        self.is_stopped = True

        # Sending an empty request to the stop service
        self.stop_client.call_async(Empty.Request())

        # Warning the user that stop was triggered
        self.get_logger().warn("Emergency Stop Activated. Now grasp commands are LOCKED.")

    # Switching the hand back to play mode
    def resume_play(self):

        # Setting the state to False
        self.is_stopped = False

        # Sending an empty request to the play service
        self.play_client.call_async(Empty.Request())

        # Informing the user that play mode is active
        self.get_logger().info("MIA Hand is set to PLAY mode. Now grasp commands are UNLOCKED.") 

# Executing the main function logic
def main():

    # Starting the ROS 2 system
    rclpy.init()

    # Creating an instance of the hand controller
    hand = MiaPhysicalController()

    # Running the node in a separate background thread
    thread = threading.Thread(target=rclpy.spin, args=(hand,), daemon=True)

    # Commencing the background thread execution
    thread.start()

    # Formatting the user interface menu
    menu = """
    ===========================================
    MIA Physical Hand Research Interface Menu
    ===========================================
    Current Settings: Aperture: {c}% | Speed: {s}%
    -------------------------------------------
    1: Cylindrical Grasp (Power)
    2: Pinch Grasp (Precision)
    3: Lateral Grasp (Key)
    4: Open/Relax Hand (0%)
    0: Exit

    C: Change Parameters (Aperture % / Speed %)
    P: Resume (Play)
    S: Emergency Stop
    -------------------------------------------
    Selection: """

    # Running the interactive command loop
    try:

        # Monitoring user input continuously
        while True:

            # Added a status visualizer to the menu
            status_text = "LOCKED (STOPPED)" if hand.is_stopped else "READY (PLAYING)"

            # Displaying menu and capturing uppercase input
            choice = input(menu.format(c=hand.close_percent, s=hand.force_speed, status=status_text)).upper()

            # Activating cylindrical grasp
            if choice == '1':
                
                hand.send_grasp('cylindrical')

            # Activating pinch grasp
            elif choice == '2':
                
                hand.send_grasp('pinch')

            # Activating lateral grasp
            elif choice == '3':
                
                hand.send_grasp('lateral')

            # Performing the relaxation sequence
            elif choice == '4':

                # Storing current closing/aperture value
                temp_close = hand.close_percent

                # Forcing the value to zero
                hand.close_percent = 0 

                # Requesting the relax grasp
                hand.send_grasp('relax')

                # Restoring the previous value
                hand.close_percent = temp_close

            # Modifying research parameters
            elif choice == 'C':

                # Validating numeric entry for parameters
                try:

                    # Prompting for new aperture percentage
                    aperture = int(input("Enter Aperture Percent (0-100): "))

                    # Prompting for new speed percentage
                    speed = int(input("Enter Speed/Force Percent (0-99): "))

                    # Constraining values within safe bounds
                    hand.close_percent = max(0, min(100, aperture))

                    # Limiting effort within safe bounds
                    hand.force_speed = max(0, min(99, speed))

                # Handling non-numeric input errors
                except ValueError:
                    
                    print("Invalid numeric input.")

            # Calling the resume service
            elif choice == 'P':
                
                hand.resume_play()

            # Calling the stop service
            elif choice == 'S':
                
                hand.emergency_stop()

            # Breaking the loop to exit
            elif choice == '0':
                
                break

    # Catching manual exit commands
    except KeyboardInterrupt:

        # Passing through to the finally block
        pass

    # Ensuring safe shutdown of the system
    finally:

        # Stopping the hand for safety
        hand.emergency_stop()

        # Closing down the ROS 2 environment
        rclpy.shutdown()


# Checking for direct script execution
if __name__ == '__main__':

    # Starting the application
    main() 
