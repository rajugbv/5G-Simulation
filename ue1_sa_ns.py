import zmq
from ctypes import cdll
import os

# Load libc and setns for namespace switching
libc = cdll.LoadLibrary('libc.so.6')
setns = libc.setns

# Namespace context manager to switch to 'ue1'
class Namespace:
    def __init__(self, nsname=None):
        self.mypath = f'/proc/{os.getpid()}/ns/net'
        self.targetpath = f'/var/run/netns/{nsname}'

        if not self.targetpath:
            raise ValueError('Invalid namespace')

    def __enter__(self):
        self.myns = open(self.mypath)  # Save current namespace
        with open(self.targetpath) as fd:
            setns(fd.fileno(), 0)  # Switch to target namespace

    def __exit__(self, *args):
        setns(self.myns.fileno(), 0)  # Restore original namespace
        self.myns.close()

def subscriber():
    with Namespace(nsname='ue1'):  # Switch to 'ue1' namespace
        context = zmq.Context()
        socket = context.socket(zmq.SUB)  # Create ZMQ SUB socket

        server_address = "tcp://172.16.0.2:6661"
        socket.bind(server_address)  # Similar to iperf -s
        print(f"Subscriber started at {server_address}")

        # Subscribe to all topics (empty filter)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Continuously receive and print messages
        while True:
            message = socket.recv_string()
            print(f"Received from publisher: {message}")

if __name__ == '__main__':
    subscriber()

