from enum import Enum
from typing import Any
import serial
import socket
import queue
import threading
import time

class CommandCode(Enum):
    ACK = 1
    STRING_CMD = 2
    INT_CMD = 3
    # ... other command codes

class SerialIF():
    class SerialStreamTypes(Enum):
        UDP_STREAM = 0
        UART_STREAM = 1

    class SerialErrorTypes(Enum):
        STREAM_CLOSE_FAIL = -3
        STREAM_INIT_FAIL = -2
        NO_TARGET = -1
        OK = 0

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
            # ... other command handlers
        }

    def __call__(self, command_code, arg = None):
        if arg is not None:
            self.command_handlers[command_code](arg)
        else:
            self.command_handlers[command_code]()

    def init_stream(self) -> SerialErrorTypes:
        pass
    def close_stream(self) -> SerialErrorTypes:
        pass
    def read_stream_bytes(self) -> bytes:
        pass
    def write_stream_bytes(self, data:bytes) -> SerialErrorTypes:
        self.tx_byte_count+=len(data)

    def write_string_cmd(self, input_string) -> SerialErrorTypes:
        message = "\x07"
        message = message+str(input_string) # Append the string
        return self.write_stream_bytes(message.encode())
    
    def write_ack(self) -> SerialErrorTypes:
        message = "\x06"
        return self.write_stream_bytes(message.encode())


class SerialUDPIF(SerialIF):
    def __init__(self) -> None:
        super().__init__()
        self.stream_type = self.SerialStreamTypes.UDP_STREAM
        self.udp_ip = "192.168.0.160"
        self.udp_port = 12345
        self.target_addr = None

    def udp_rstream_task(self):
        while True:
            try:
                data, addr = self.udp_sock.recvfrom(1024)
                self.data_queue.put((data,addr))
            except:
                break
        self.control_queue.put(-1)

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
            # Wait of the initial message of the FC
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
        
# Check if the script is being run as the main program
if __name__ == "__main__":

    # Timeout for the command write
    write_timeout = 2

    # Runtime flags
    wait_for_ack = False
    write_timestamp = time.time()

    # Commands to queue up
    command_string = "This is a string command, can you hear me? I'm making this a longer message for funsies"
    command_queue = []
    command_queue.extend([ (CommandCode.STRING_CMD,command_string) for i in range(100) ])
    # command_queue.extend([ (CommandCode.ACK,None) for i in range(5) ])

    start_time = time.time()

    # Init the serial stream
    udp_stream = SerialUDPIF()
    udp_stream.init_stream(wait_for_ack=True)

    # Loop until all commands have been sent and acknowledged
    for command, arg in command_queue:

        # Get the next command from the queue
        if command == CommandCode.STRING_CMD:
            udp_stream(command,arg)
        else:
            udp_stream(command)

        write_timestamp = time.time()
        while True:

            # Read the stream for incoming messages
            data = udp_stream.read_stream_bytes()

            # If a message is received
            if not (data==b''):
                print(f"Received message: {data.decode()}")
                break

            # If the command has not been acknowledged within the timeout period, try again
            if ( (time.time()-write_timestamp) > write_timeout ):
                print("Timing out...")
                break

    stop_time = time.time()

    runtime = stop_time-start_time
    throughput = udp_stream.tx_byte_count/runtime

    print(f"Sent {udp_stream.tx_byte_count} bytes in {round(runtime,2)}s")
    print(f"Throughput: {round(throughput,2)}B/s")

    # Close the serial stream
    udp_stream.close_stream()

