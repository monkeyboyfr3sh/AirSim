from enum import Enum
from typing import Any
import serial
import socket
import queue
import threading
import time
import random
import string
import csv
import numpy as np
import os

# Define an enumeration for command codes
class CommandCode(Enum):
    ACK = 1
    STRING_CMD = 2
    INT_CMD = 3
    # ... other command codes

# Define a base class for serial communication interfaces
class SerialIF():
    # Define enumerations for stream types and error types
    class SerialStreamTypes(Enum):
        UDP_STREAM = 0
        UART_STREAM = 1

    class SerialErrorTypes(Enum):
        STREAM_CLOSE_FAIL = -3
        STREAM_INIT_FAIL = -2
        NO_TARGET = -1
        OK = 0

    # Constructor
    def __init__(self) -> None:
        self.stream_type = None
        self.stream_thread = None
        self.control_queue = queue.Queue()
        self.data_queue = queue.Queue()
        self.tx_byte_count = 0

        # Define a dictionary of command handlers
        self.command_handlers = {
            CommandCode.ACK: self.write_ack,
            CommandCode.STRING_CMD: lambda x: self.write_string_cmd(x),
            CommandCode.INT_CMD: lambda x: self.write_stream_bytes(bytes(x)),
            # ... other command handlers
        }

    # Callable function to execute command handlers
    def __call__(self, command_code, arg = None):
        if arg is not None:
            self.command_handlers[command_code](arg)
        else:
            self.command_handlers[command_code]()

    # Clear number of bytes sent
    def clear_byte_count(self):
        self.tx_byte_count = 0

    # Abstract methods for handling streams
    def init_stream(self) -> SerialErrorTypes:
        pass
    def close_stream(self) -> SerialErrorTypes:
        pass
    def read_stream_bytes(self) -> bytes:
        pass
    def write_stream_bytes(self, data:bytes) -> SerialErrorTypes:
        self.tx_byte_count+=len(data)

    # Method to write a string command
    def write_string_cmd(self, input_string) -> SerialErrorTypes:
        message = "\x07"
        message = message+str(input_string) # Append the string
        return self.write_stream_bytes(message.encode())

    # Method to write an acknowledgement
    def write_ack(self) -> SerialErrorTypes:
        message = "\x06"
        return self.write_stream_bytes(message.encode())
        
class SerialUDPIF(SerialIF):
    # Constructor
    def __init__(self) -> None:
        super().__init__()
        self.stream_type = self.SerialStreamTypes.UDP_STREAM
        self.udp_ip = "192.168.0.160"
        self.udp_port = 12345
        self.target_addr = None

    # Thread function to receive data from the UDP socket
    def udp_rstream_task(self):
        while True:
            try:
                data, addr = self.udp_sock.recvfrom(1024)
                self.data_queue.put((data,addr))
            except:
                break
        self.control_queue.put(-1)

    # Initialize the UDP stream
    def init_stream(self, wait_for_ack = False, stream_timeout=0.5) -> SerialIF.SerialErrorTypes:
        super().init_stream()

        # Create a UDP socket
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.bind((self.udp_ip, self.udp_port))
        except:
            return SerialIF.SerialErrorTypes.STREAM_INIT_FAIL

        # Fork a thread to receive data from this socket
        self.stream_thread = threading.Thread(
            target=self.udp_rstream_task)
        self.stream_thread.start()

        if(wait_for_ack):
            # Wait for the initial message of the FC
            while (self.read_stream_bytes() == b''):
                pass
            # Send a message
            message = f"Hello friend, rx_count = {0}"
            self.write_stream_bytes(message.encode())
            # Clear the queue
            read_timestamp = time.time()
            while ( (time.time()-read_timestamp) < stream_timeout ):
                if not (self.read_stream_bytes() == b''):
                    read_timestamp = time.time()
        return SerialIF.SerialErrorTypes.OK

    def close_stream(self) -> SerialIF.SerialErrorTypes:
        super().close_stream()
        try:
            self.udp_sock.close()
        except:
            return SerialIF.SerialErrorTypes.STREAM_CLOSE_FAIL

        return SerialIF.SerialErrorTypes.OK

    def read_stream_bytes(self) -> bytes:
        super().read_stream_bytes()
        if not (self.data_queue.empty()):
            data,addr = self.data_queue.get()
            self.target_addr = addr
            return data
        else:
            return bytearray()

    def write_stream_bytes(self, data:bytes ) -> SerialIF.SerialErrorTypes:
        super().write_stream_bytes(data)
        if (self.target_addr!=None):
            self.udp_sock.sendto(data,self.target_addr)
            return SerialIF.SerialErrorTypes.OK
        else:
            return SerialIF.SerialErrorTypes.NO_TARGET

def run_udp_stream(serial_stream:SerialIF, command_queue, write_timeout=2):
    # Runtime flags
    start_time = time.time()
    
    # Begginning message
    print(f"Running UDP stream with {len(command_queue)} commands of length {len(command_queue[0][1])}...")

    latency_list = []

    # Loop until all commands have been sent and acknowledged
    for command, arg in command_queue:

        # Get the next command from the queue
        if command is not None:
            serial_stream(command,arg)
        else:
            serial_stream(command)

        # Start of transmission
        write_timestamp = time.time()

        while True:

            # Read the stream for incoming messages
            data = serial_stream.read_stream_bytes()

            # If a message is received
            if not (data==b''):
                latency_list.append(time.time()-write_timestamp)
                print(f"Received message: {data.decode()}",end='\r')
                break

            # If the command has not been acknowledged within the timeout period, try again
            if ( (time.time()-write_timestamp) > write_timeout ):
                print("Timing out...")
                break

    average_latency_ms = np.asarray(latency_list).mean()*1000

    # Calculate throughput
    runtime = time.time() - start_time
    throughput = serial_stream.tx_byte_count / runtime
    throughput_KBs = throughput / 1024

    return runtime, throughput_KBs, average_latency_ms

def generate_random_string(length):
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(length))

def create_data_file(num_commands, payload_len):
    print(f"Creating a payload file for {num_commands} commands, of size {payload_len}... ",end='')
    # Generate commands and write to file
    command_queue = []
    relative_path = os.path.join("./data",f"commands_{num_commands}_{payload_len}.txt")
    with open(relative_path, 'w') as f:
        for i in range(num_commands):
            command_string = generate_random_string(payload_len-1)
            command_queue.append((CommandCode.STRING_CMD, command_string))
            f.write(command_string + '\n')
    print("Done!")

# Check if the script is being run as the main program
if __name__ == "__main__":

    LOG_FNAME = "throughput_latency.csv"
    wr_relative_path = os.path.join("./data",LOG_FNAME)

    # Payload sizes to write/read
    payload_sizes = [32,128,512,1024,2048,4096,8192,16384,32768]    

    # for payload_size in payload_sizes:
    #     # File to create
    #     NUM_COMMANDS = 1000
    #     PAYLOAD_LEN = payload_size
    #     create_data_file(NUM_COMMANDS,PAYLOAD_LEN)

    # Init the serial stream
    udp_stream = SerialUDPIF()
    udp_stream.init_stream(wait_for_ack=True)

    # Loop over payload sizes
    for payload_size in payload_sizes:
        # Read commands from file
        rd_relative_path = os.path.join("./data",f"commands_1000_{payload_size}.txt")
        with open(rd_relative_path, 'r') as f:
            command_queue = [(CommandCode.STRING_CMD, line.strip()) for line in f.readlines()]

        # Run UDP stream and measure throughput
        runtime, throughput_KBs, average_latency_ms = run_udp_stream(udp_stream, command_queue, write_timeout=2)
        throughput_MBs = throughput_KBs / 1024

        # Print results
        print(f"Runtime: {round(runtime,2)} s; Throughput: {round(throughput_KBs,2)} KB/s; {round(throughput_MBs,2)} MB/s; average latency {round(average_latency_ms,2)} ms")

        # Open CSV file for appending
        with open(wr_relative_path, mode='a', newline='') as f:
            writer = csv.writer(f)

            # Append throughput to CSV file
            writer.writerow([payload_size, throughput_KBs, average_latency_ms])

        # Close the serial stream
    udp_stream.close_stream()

    # Print confirmation message
    print(f'Results written to {LOG_FNAME}')