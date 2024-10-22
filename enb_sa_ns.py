import zmq
import time
import sys

def publisher():
    
    context = zmq.Context()
    socket = context.socket(zmq.PUB)

    server_address = "tcp://172.16.0.2:6661"  
    try:
        socket.connect(server_address)  
        print(f"Publisher connected to {server_address}")

        while True:
            message = "Hello from main IP 172.16.0.1!"
            print(f"Publishing: {message}")
            socket.send_string(message)

    except zmq.ZMQError as e:
        print(f"Error: {e}")
    finally:
        # Ensure proper cleanup
        socket.close()
        context.term()

if __name__ == '__main__':
    try:
        publisher()
    except KeyboardInterrupt:
        print("Publisher terminated.")

