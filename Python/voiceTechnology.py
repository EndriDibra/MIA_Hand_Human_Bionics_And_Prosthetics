# Author: Endri Dibra 

# Project: Voice Controlled ROS 2 Pipeline Interface

# Description: Host speech recognition bridge forwarding
# wake up words and target objects to a Docker container via TCP sockets
# with interactive two-way voice-to-text and text-to-voice interfaces

# Importing socket library
# for network communication
import socket

# Importing speech recognition
# library for audio processing
import speech_recognition as sr 

# Importing text to speech
# library for verbal system responses
import pyttsx3

# Importing time library
# for managing execution delays
import time


# Setting the local host
# IP address for the target container
HOST_IP = '127.0.0.1'

# Setting the target port
# number for socket connection
PORT = 5001

# Initializing the text to
# speech synthesis subsystem engine
engine = pyttsx3.init()

# Setting the vocal delivery
# speech rate to a natural cadence
engine.setProperty('rate', 165)

# Formatting system status output
# into local audio speaker broadcasts
def speak(text):

    # Printing the textual copy
    # of the spoken message out
    print(f"[System Voice]: {text}")

    # Queueing the string message
    # for vocal track rendering
    engine.say(text)

    # Executing the speech tasks
    # blocking execution until completed
    engine.runAndWait()

# Processing the message
# delivery to the Docker container
def send_to_container(message):

    # Monitoring network operations
    # for connection or transmission faults
    try:

        # Initializing an internet streaming socket instance
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Establishing a TCP connection
        # with the target socket server
        s.connect((HOST_IP, PORT))

        # Sending the encoded byte string
        # across the established network link
        s.sendall(message.encode())

        # Terminating the network
        # socket connection safely
        s.close()

        # Returning true to signal
        # a successful payload delivery
        return True

    # Handling socket exceptions
    # or connection dropouts
    except Exception as e:

        # Printing the specific network connection failure message
        print(f"Connection to Docker failed: {e}")

        # Returning false to signal
        # a transmission breakdown
        return False

# Executing the main function
# to run the voice interface
def main():

    # Constructing the audio recognition instance
    recognizer = sr.Recognizer()

    # Disabling the dynamic threshold
    # adjustments to enforce static tuning
    recognizer.dynamic_energy_threshold = False 

    # Defining the minimum acoustic
    # energy required to trigger listening
    recognizer.energy_threshold = 200  

    # Specifying the continuous silence
    # duration required to conclude a phrase
    recognizer.pause_threshold = 2.0  

    print("Host Voice Interface Active")
    
    # Defining acceptable words for the primary activation keyword
    WAKE_WORDS = {'go', 'start', 'begin'}
    
    # Specifying valid objects registered
    # within the downstream vision node
    VALID_OBJECTS = {'bottle', 'cup'}

    # Defining terms used to trigger
    # a clean interface shutdown
    EXIT_WORDS = {'exit', 'stop', 'quit'}

    # Monitoring the acoustic space
    # within an infinite loop
    while True:

        # Printing the notification for ambient noise evaluation
        print("\nPhase 1: Calibrating room noise... Please wait 1 second.")

        # Interfacing with the default
        # system audio capturing hardware
        with sr.Microphone() as source:

            # Adjusting the recognition ceiling
            # based on ambient environment samples
            recognizer.adjust_for_ambient_noise(source, duration=1.0)
            
            print("Ready! Say an activation phrase (e.g., 'go system', 'start the system') or 'exit'...")

            # Monitoring microphone input
            # streams for wake token matching
            try:

                # Capturing speech fragments with a fixed duration constraint
                audio = recognizer.listen(source, timeout=None, phrase_time_limit=5.0)

                # Querying Google web services to convert captured acoustics to text
                text = recognizer.recognize_google(audio).lower().strip()

                print(f"Captured Wake up Text: '{text}'")

            # Handling cases where speech patterns
            # cannot be resolved to text
            except sr.UnknownValueError:

                print("Audio detected but phrase not understood. Let's try again...")

                # Continuing execution at
                # the top of the loop to try again
                continue

            # Catching generic hardware reading 
            # exceptions or driver failures
            except Exception as e:

                # Printing the hardware driver
                # or streaming fault description
                print(f"Microphone read error: {e}")

                # Suspending loop execution briefly
                # before attempting a recovery
                time.sleep(1)

                # Continuing execution to
                # restart calibration steps
                continue

        # Checking if an exit command was spoken during Phase 1
        if any(word in text for word in EXIT_WORDS):

            # Announcing terminal system closing response
            speak("Good bye!")

            # Breaking the operation loop
            # terminating the application cleanly
            break

        # Checking if any monitored wake up phrase tokens exist inside the text
        if not any(word in text for word in WAKE_WORDS):

            print("Ignored. Wake up keyword not found in the phrase.")

            # Continuing execution back
            # to ambient noise checking blocks
            continue

        # Announcing interactive conversational response for system startup
        speak("Hello, how are you? Starting the system!")

        print("Sending 'go' activation signal to ROS2 container...")

        # Transmitting the standardized
        # activation command across the socket
        if not send_to_container("go"):

            # Continuing back to baseline
            # phase if socket transfer fails
            continue
            
        # Suspending operations to allow
        # the remote node to change state
        time.sleep(1.5) 

        print("\nPhase 2: Please state target object phrase (e.g., 'find the bottle', 'get the cup'):")
        
        # Opening the microphone hardware
        # interface for target object collection
        with sr.Microphone() as source:

            print("Calibrating for target entry...")

            # Tuning ambient noise thresholdsover a abbreviated sample window
            recognizer.adjust_for_ambient_noise(source, duration=0.8)
            
            print("Listening for object phrase...")

            # Monitoring microphone data
            # specifically for target classification
            try:

                # Acquiring the secondary speech sequence with strict time caps
                audio_target = recognizer.listen(source, timeout=10.0, phrase_time_limit=10.0)

                # Converting the raw target acoustics to clean lowercase text
                target_phrase = recognizer.recognize_google(audio_target).lower().strip()

                print(f"Captured Target Text: '{target_phrase}'")

                # Checking if an exit command was spoken during Phase 2
                if any(word in target_phrase for word in EXIT_WORDS):

                    # Announcing terminal system closing response
                    speak("Good bye!")

                    # Terminating the loop structure
                    break
                
                # Initializing the container to
                # hold the filtered target token
                found_target = None

                # Iterating over valid objects to
                # locate substrings inside the phrase
                for obj in VALID_OBJECTS:

                    # Checking if the current valid keyword
                    # is wrapped within the speech string
                    if obj in target_phrase:

                        # Assigning the matching token
                        # to the tracking container
                        found_target = obj

                        # Breaking out of the object
                        # search loop upon first match
                        break 
                
                # Checking if a valid manipulation
                # target was extracted
                if found_target:

                    # Announcing interactive conversational status for object assignment
                    speak(f"I am going to send the object: {found_target}")

                    print("Forwarding target object to ROS2...")

                    # Pushing the isolated object label
                    # across the TCP socket pipeline
                    send_to_container(found_target)

                    # Delaying execution to
                    # provide an interface rest window
                    time.sleep(2.0) 

                    # Announcing interactive conversational response before loop reset
                    speak("Of course! Restarting the system.")

                # Handling cases where speech content
                # contains invalid object names
                else:

                    print(f"Phrase ignored. Could not extract a valid object from: '{target_phrase}'")
                
            # Catching occurrences where no voice
            # input is detected inside the timeout
            except sr.WaitTimeoutError:

                print("Object selection timed out (no speech detected). Resetting to baseline...")

            # Handling failures to resolve words
            # within the target audio stream
            except sr.UnknownValueError:

                print("Could not decipher object phrase text. Resetting to baseline...")

            # Catching fallback errors during
            # target collection processing
            except Exception as e:

                # Printing generic description
                # logs tracking the phase fault
                print(f"Error capturing target phase: {e}")

        # Checking if an exit request broke the inner targeting statement blocks
        if any(word in target_phrase for word in EXIT_WORDS if 'target_phrase' in locals()):

            # Terminating the application loop completely
            break


# Direct script execution
if __name__ == "__main__":

    # Invoking the main function
    main() 
