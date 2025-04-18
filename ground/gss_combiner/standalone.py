# Runs a standalone version of the MQTT combiner software (Only supports one COM port)
from datetime import datetime, timezone
from sys import getsizeof
import json
import copy
import time
import threading
import sys
import queue

import serial # PySerial
import paho.mqtt.client as mqtt
import time

import argparse

import util.logger

stdin_q = queue.Queue()

def read_stdin():
    for line in sys.stdin:
        stdin_q.put(line)

threading.Thread(target=read_stdin, daemon=True).start()

class TelemetryStandalone():
    """A thread class handling all communications between COM ports to which telemetry devices are connected."""
    def __init__(self, com_port, server_uri, stage, should_log) -> None:
        self.__comport: serial.Serial = serial.Serial(com_port, baudrate=38400, write_timeout=1, timeout=0.05)
        self.__comport.reset_input_buffer()

        self.__should_log = should_log
        self.__outfile = None
        self.__outfile_raw = None

        if(self.__should_log):
            self.__outfile = open(f"./outputs/{time.time()}_log.telem", "w+")
            self.__outfile_raw = open(f"./outputs/{time.time()}_raw_log.txt", "w+")

        print("Pre-start diagnostics:")
        print("Logging? ", self.__should_log)
        print("Using COM port ", com_port)
        print("Publishing @ ", server_uri)
        print("Interpreting data as", stage)
        print()

        self.__uri = server_uri
        self.__mqttclient: mqtt.Client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        print("Connecting to broker @ " + str(self.__uri))
        try:
            self.__mqttclient.connect(self.__uri, port=1884)
        except:
            print("UNABLE TO CONNECT TO ", self.__uri)
            print("REPORT_ERR", flush=True)
            sys.exit(1) 
        print("Subscribing to MQTT streams...")

        self.__data_channel = "FlightData-" + stage
        self.__control_channel = "Control-" + stage


        def on_message(client, userdata, msg): 
            """Callback function that will be called every time the client receives a message from the broker (something is added to the topic, commands to be sent)"""
            payload_str = msg.payload.decode()
            try:
                payload_obj = json.loads(payload_str)
                type = payload_obj["type"]

                if(type != "telemetry_command"):
                    return
                raw_cmd = payload_obj["raw"]
            except:
                print(f"Failed to decode Command stream packet: {payload_str}")

            print(f"Recieved command '{raw_cmd}', acknowledged & sent to telemetry threads.")
            # print(f"Recieved command '{raw_cmd}', acknowledged & sent to telemetry threads.")

            ack_msg = {"type": "acknowledge_combiner", "ch": self.__control_channel, "cmd_ack": raw_cmd}
            self.__external_commands.append(raw_cmd)
            self.__mqttclient.publish(msg.topic, json.dumps(ack_msg))

        self.__mqttclient.on_message = on_message

        self.__mqttclient.subscribe(self.__control_channel) 
        print("Control channel: ", self.__control_channel)

        print("MQTT systems initialized.")

        self.__rf_set = True
        self.__rf_freq = 0
        self.__last_rf_command_sent = datetime.now().timestamp()
        self.__rf_command_period = 3

        self.__consecutive_good = 0

        self.__external_commands = []

        self.__in_buf = ""

        self.__out_all = False


    def process_packet(self, packet_json):
        """Append metadata to a packet to conform to GSS v1.1 packet structure"""
        # Incoming packets are of form {"type": "data", value: { ... }}
        time = datetime.now(timezone.utc)

        # Append timestamps :)
        return {'value': packet_json['value'], 'type': packet_json['type'], 'utc': str(time), 'unix': datetime.timestamp(time), 'src': self.__comport.name}
    
    def write_frequency(self, frequency: float):
        """Send a FREQ command to the associated telemetry device to change frequencies, then wait for a response."""
        print("Writing frequency command (FREQ:" + str(frequency) + ")")
        try:
            self.__send_comport("FREQ:" + str(frequency) + "\n")
            self.__rf_freq = frequency

            self.__rf_set = False

            self.__last_rf_command_sent = datetime.now().timestamp()
        except Exception as e:
            print("Unable to send FREQ command: ", e, "Continuing with no FREQ change.")

    def __send_comport(self, msg: str):
        """Send arbirtary data to the associated COM port"""
        self.__comport.write(msg.encode())

    def __read_comport(self):
        """Read all data from the associated COM port"""
        if self.__comport.in_waiting:
            p_full = self.__in_buf
            while self.__comport.in_waiting:
                data = self.__comport.read_until("\n")
                p_full += bytes.decode(data, encoding="ascii")

            p_data = p_full.split("\n")
            self.__in_buf = p_data.pop()
                
            return p_data
        else:
            return []

        
    def run(self) -> None:
        print("Starting background systems...")
        self.__mqttclient.loop_start()
        print(f"REPORT_OK:{self.__uri}", flush=True)
        print("\nReady!\n", flush=True)
        while True:
            # Wrap all in a try-except to catch errors.

            try:
                if not self.__rf_set and datetime.now().timestamp() - self.__last_rf_command_sent >= self.__rf_command_period:
                    # Send a new frequency command if the last one went unacknowledged
                    self.write_frequency(self.__rf_freq)
                    
                # Read all from the comport
                packets = self.__read_comport()

                if len(packets) > 0:
                    if self.__out_all:
                        for p in packets:
                            print(f"[F] {p.strip()}", flush=True)

                # Process stdin
                if not stdin_q.empty():
                    line = stdin_q.get().strip()
                    is_internal = False
                    if line == "HELP":
                        is_internal = True
                        print("[CMD] HELP : Send commands to the combiner and feather.\nHELP - Show this screen\nPING - Test command\nOUT_ALL - Output all serial input\nOUT_DEFAULT - (Default setting) only output default combiner data", flush=True)

                    if line == "PING":
                        is_internal = True
                        print("[CMD] PONG", flush=True)

                    if line == "OUT_ALL":
                        is_internal = True
                        self.__out_all = True
                        print("[CMD] OUT_ALL - Now outputting all serial data.")
                        print("Note: Serial data from the feather will be formatted like below:")
                        print("[F] This is a sample serial output.", flush=True)

                    if line == "OUT_DEFAULT":
                        is_internal = True
                        self.__out_all = False
                        print("[CMD] OUT_DEFAULT - Now outputting only combiner data.", flush=True)
                        
                    if not is_internal:
                        print("[CMD] Non internal command, forwarding to feather.", flush=True)
                        self.__external_commands.append(line)


                # Write if needed to comport
                if(len(self.__external_commands) > 0):
                    for cmd in self.__external_commands:
                        print(f"[TO FEATHER] '{cmd}'", flush=True)
                        self.__send_comport(str(cmd) + "\r\n")
                    self.__external_commands = []

                if(len(packets) == 0):
                    # Defer if no data in COM port
                    continue

                for pkt_r in packets:
                    pkt = pkt_r.rstrip() # Strip whitespace characters
                    if(len(pkt) == 0):
                        continue # Ignore empty data
                    try:
                        packet_in = json.loads(pkt)
                        
                        # print(str(packet_in))
                        if type(packet_in) == int:
                            # print("Read int? Discarding")
                            continue
                        if type(packet_in) == float:
                            # print("Read float? Discarding")
                            continue

                        if type(packet_in) != dict:
                            # print("Read non dict?" + str(pkt) + " type " + str(type(packet_in)))
                            continue

                        if(self.__should_log):
                            self.__outfile_raw.write(f"{packet_in['type']}: " + str(packet_in) + "\n")
        
                        if packet_in['type'] == "command_success":
                            print("[RX] Command good")
                            packet_ack_encoded = json.dumps(packet_in).encode("utf-8")
                            self.__mqttclient.publish(self.__control_channel, packet_ack_encoded)
                            continue

                        # This is jank but idc
                        if packet_in['type'] == "bad_command":
                            print("[RX] Command bad")
                            packet_ack_encoded = json.dumps(packet_in).encode("utf-8")
                            self.__mqttclient.publish(self.__control_channel, packet_ack_encoded)
                            continue

                        if packet_in['type'] == "command_acknowledge":
                            print("[RX] Command ACK")
                            packet_ack_encoded = json.dumps(packet_in).encode("utf-8")
                            self.__mqttclient.publish(self.__control_channel, packet_ack_encoded)
                            continue

                        if packet_in['type'] == "command_sent":
                            print("[RX] Command send confirmed")
                            packet_ack_encoded = json.dumps(packet_in).encode("utf-8")
                            self.__mqttclient.publish(self.__control_channel, packet_ack_encoded)
                            continue

                        # print(packet_in)
                        if not self.__rf_set:
                            # Wait for freq change and periodically send new command to try to change freq.

                            if packet_in['type'] == "freq_success":
                                if float(packet_in['frequency']) == float(self.__rf_freq):
                                    print("Frequency set: Listening on " + str(self.__rf_freq))
                                    self.__rf_set = True
                                else:
                                    print("Recieved incorrect frequency!")
                                    continue    
                            if packet_in['type'] == "command_success":
                                print("Successful command")
                            else:
                                print("Recieved packet from wrong stream.. Discarding due to freq change.")
                                continue
                        
                        # # No more logging heartbeats
                        # if packet_in['type'] == 'data':
                        #     print("OK ", end="\r")

                    except json.decoder.JSONDecodeError as json_err:
                        # print(f"Recieved corrupted JSON packet. Flushing buffer.")
                        print(f"E[{len(pkt)}]")
                        self.__consecutive_good = 0
                        continue

                    # Process and queue the packet
                    processed = self.process_packet(packet_in)

                    # Add packet metadata
                    packet_new = {
                        "data": processed,
                        "metadata": {
                            "raw_stream": self.__data_channel,
                            "time_published": time.time()
                        }
                    }

                    # Send to MQTT
                    data_encoded = json.dumps(packet_new).encode("utf-8")

                    if processed['value']['is_sustainer'] == True:
                        data_channel = "FlightData-Sustainer"
                    else:
                        data_channel = "FlightData-Booster"

                    #log
                    if(self.__should_log):
                        self.__outfile.write(json.dumps(packet_new) + "\n")

                    try:
                        self.__mqttclient.publish(data_channel, data_encoded)
                        print("PACKETS GOOD: ", self.__consecutive_good, end="   \r")
                        self.__consecutive_good += 1
                    except Exception as e:
                        print("Failed to publish!")
                    

                    
            except Exception as e:
                try:
                    print(f"[Telem {self.__comport.name}] Ran into an uncaught exception.. continuing gracefully.") # Always print these.
                    print(e)
                    # print(f"Error dump:", traceback.format_exc(e))
                except Exception as e:
                    print("Exception while handling exception!")


def parse_params(arguments):
    arg_parser = argparse.ArgumentParser(
        prog='GSS Combiner (Standalone)',
        description="A system to take telemetry from different sources and pass it onto a MQTT broker for ground station visualization"
    )

    arg_parser.add_argument("--port", type=str, help="Selects which COM port to use")

    arg_parser.add_argument("--booster", action="store_true", help="Should we use booster?")
    arg_parser.add_argument("--sustainer", action="store_true", help="Should we use sustainer?")
    arg_parser.add_argument("--duo", action="store_true", help="Should we use feather duo?")

    arg_parser.add_argument("-n", "--no-log", action="store_true", help="Will not log data to logfiles for this run")
    arg_parser.add_argument("-i", "--ip", type=str, help="Connects to a specific IP. (Defaults to localhost)")
    
    args = arg_parser.parse_args(arguments)

    if(args.booster and args.sustainer):
        print("Cannot specify both booster and sustainer")
        exit(1)

    should_log = not args.no_log

    source = "__none"
    if args.booster:
        source = "Booster"

    if args.sustainer:
        source = "Sustainer"

    if args.duo:
        source = "Multistage (Sustainer / Booster)"
        print("Disregarding any booster/sustainer commands. Initializing as Feather Duo.", flush=True)

    ip = "localhost"
    if args.ip:
        ip = args.ip

    return source, should_log, ip, args.port

if __name__ == "__main__":

    stage, should_log, ip, port = parse_params(sys.argv[1:])

    t = TelemetryStandalone(port, ip, stage, should_log)

    t.run()
