import socket
import json
import time
import os

def send_command_and_receive_response(command, host, port):
    try:
        # Create a TCP/IP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            # Connect to the server
            sock.connect((host, port))
            
            # Send the command
            sock.sendall(command.encode('utf-8'))
            
            # Receive the response
            response = sock.recv(4096).decode('utf-8')
            
            # Parse the JSON response
            response_json = json.loads(response)
            return response_json
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

if __name__ == "__main__":
    HOST = "192.168.10.10"  # Replace with the server's IP address
    PORT = 31001        # Replace with the server's port number
    COMMAND = "/api/joy_control"

    print("The program will automatically move forward and backward.")
    os.system('stty -echo')  # Disable terminal echo

    try:
        while True:
            # Forward movement
            param_str = "angular_velocity=0&linear_velocity=0.1"
            cmd = COMMAND + "?" + param_str + "&uuid=123456"
            response = send_command_and_receive_response(cmd, HOST, PORT)
            if response:
                print("Moving forward:")
                print(json.dumps(response, indent=4))
            else:
                print("Failed to get a valid response while moving forward.")

            time.sleep(2)  # Move forward for 2 seconds

            # Backward movement
            param_str = "angular_velocity=0&linear_velocity=-0.1"
            cmd = COMMAND + "?" + param_str + "&uuid=123456"
            response = send_command_and_receive_response(cmd, HOST, PORT)
            if response:
                print("Moving backward:")
                print(json.dumps(response, indent=4))
            else:
                print("Failed to get a valid response while moving backward.")

            time.sleep(2)  # Move backward for 2 seconds

    except KeyboardInterrupt:
        print("Exiting...")

    os.system('stty echo')  # Re-enable terminal echo
