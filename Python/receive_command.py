# Author: Endri Dibra 

# Project: This Linux-Ubuntu OS script listens for incoming text command inputs,
# sent from a script running on Windows OS, for experimental purposes
# to test the communication between the two OSs, using TCP/IP sockets 

# Importing the socket library
# to enable network communication
import socket


# Defining the function to start the server
def start_receiver():

    # Assigning the address to listen on all available interfaces
    host = '0.0.0.0'
    
    # Specifying the port to monitor
    port = 5000
    
    # Initializing the TCP/IP socket with a context manager
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    
        # Binding the socket to the host address and port
        s.bind((host, port))
    
        # Preparing the socket to listen for incoming connections
        s.listen()
    
        # Displaying the status message for the listening port
        print(f"Listening for commands on port {port}...")
    
        # Running a loop to keep the server active
        while True:
    
            # Accepting an incoming connection request
            connection, address = s.accept()
    
            # Managing the connection lifecycle automatically
            with connection:
    
                # Receiving and decoding up to 1024 bytes of data
                data = connection.recv(1024).decode('utf-8')
    
                # Checking if the received data is empty
                if not data:
    
                    # Terminating the inner loop if no data exists
                    break
    
                # Printing the received string to the terminal
                print(f"The Received command from WINDOWS OS script is: {data}")
    
                # Comparing the data to the command string "OPEN"
                if data == "OPEN":
    
                    # Simulating the action for opening the hand
                    print("Executing: Opening hand...")
    
                # Comparing the data to the command string "CLOSE"
                elif data == "CLOSE":
    
                    # Simulating the action for closing the hand
                    print("Executing: Closing hand...")
    
                # Handling cases where the command is not recognized
                else:
    
                    # Logging the unidentified input
                    print(f"Executing unknown command: {data}")


# Running the script entry point
if __name__ == "__main__":
    
    # Invoking the server startup function
    start_receiver()