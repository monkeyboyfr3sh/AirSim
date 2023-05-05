from enum import Enum
import serial_interface as serif
import time
import queue

class CommandMessages(Enum):
    hello_command = "\x0A"
    string_1_command = '\x07AP String 1'
    string_2_command = '\x07AP String 2'

class ArdupilotIF():
    def __init__(self, serial_interface: serif.SerialIF) -> None:
        # Init the stream
        self.serial_if = serial_interface
        self.serial_if.init_stream(wait_for_ack=True)
        self.stream_queue = queue.Queue()

    def send_message(self, message , rety_count = 0, timeout = 0.5) -> bytes:
        for attempt in range(rety_count+1):
            # Send the message
            try:
                self.serial_if.write_stream_bytes(message.encode())
            except AttributeError: # Allows byes to passthrough
                self.serial_if.write_stream_bytes(message)
            # Wait for the ack
            rx_timestamp = time.time()
            while( (time.time()-rx_timestamp) < timeout):
                rx_bytes = self.serial_if.read_stream_bytes()
                if not ( rx_bytes==b''):
                    return rx_bytes # return the response if get one
        return bytearray()

if __name__ == "__main__":
    ap_if = ArdupilotIF(serif.SerialUDPIF()) 

    count = 0
    wait_for_ack = False
    write_timestamp = time.time()
    write_timeout = 2

    messages = [
        'hi', 
        CommandMessages.hello_command.value,
        CommandMessages.string_1_command.value,
        CommandMessages.string_2_command.value,
        CommandMessages.hello_command.value,
        'hi', 
        ]
    for message in messages:
        # Send the message, get the response
        response = ap_if.send_message(message)

        # Check if received response
        if not (response == b''):
            print(f"Received message: {response.decode()}")
            count+=1

    ap_if.serial_if.close_stream()