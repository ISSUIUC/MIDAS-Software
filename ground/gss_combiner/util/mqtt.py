from datetime import datetime, timezone
from sys import getsizeof
import threading
import json
import traceback
import copy
import io

import serial # PySerial
import paho.mqtt.client as mqtt

import util.logger

class TelemetryThread(threading.Thread):
    """A thread class handling all communications between COM ports to which telemetry devices are connected."""
    def __init__(self, com_port, log_stream: util.logger.LoggerStream) -> None:
        super(TelemetryThread, self).__init__(daemon=True)
        self.__log = log_stream
        self.__log.console_log(f"Opening {com_port}")

        self.__comport: serial.Serial = serial.Serial(com_port, baudrate=4800, write_timeout=1)
        self.__comport.reset_input_buffer()

        self.__log.console_log(f"Telemetry thread created.")
        self.__combiners = []

        self.__rf_set = True
        self.__rf_freq = 0
        self.__last_rf_command_sent = datetime.now().timestamp()
        self.__rf_command_period = 3

        self.__external_commands = []

        
    def process_packet(self, packet_json):
        """Append metadata to a packet to conform to GSS v1.1 packet structure"""
        # Incoming packets are of form {"type": "data", value: { ... }}
        time = datetime.now(timezone.utc)

        # Append timestamps :)
        return {'value': packet_json['value'], 'type': packet_json['type'], 'utc': str(time), 'unix': datetime.timestamp(time), 'src': self.__comport.name}
    
    def write_frequency(self, frequency: float):
        """Send a FREQ command to the associated telemetry device to change frequencies, then wait for a response."""
        self.__log.console_log("Writing frequency command (FREQ:" + str(frequency) + ")")
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
            p_full = ""
            while self.__comport.in_waiting:
                data = self.__comport.read_all()
                p_full += bytes.decode(data, encoding="ascii")
                
            
            return p_full.split("\n")
        else:
            return []
        
    def add_combiner(self, combiner):
        """Add a combiner data sink for this telemetry thread"""
        self.__combiners.append(combiner)

    def ready(self) -> bool:
        # On startup, tells the main process if this thread is ready
        return self.__rf_set
    
    def send_command(self, command: str):
        self.__external_commands.append(command)
        
    def run(self) -> None:
        self.__log.console_log(f"Telemetry thread ready.")

        while True:
            # Wrap all in a try-except to catch errors.

            try:
                if not self.__rf_set and datetime.now().timestamp() - self.__last_rf_command_sent >= self.__rf_command_period:
                    # Send a new frequency command if the last one went unacknowledged
                    self.write_frequency(self.__rf_freq)
                    
                # Read all from the comport
                packets = self.__read_comport()

                # Write if needed to comport
                if(len(self.__external_commands) > 0):
                    for cmd in self.__external_commands:
                        self.__log.console_log(f"Sending command '{cmd}'")
                        self.__send_comport(str(cmd))
                    self.__external_commands = []

                if(len(packets) == 0):
                    # Defer if no data in COM port
                    continue

                # Process raw to packet
                self.__log.console_log(f"Processing {len(packets) - 1} packets..")
                self.__log.set_waiting(len(packets) - 1)

                for pkt_r in packets:
                    pkt = pkt_r.rstrip() # Strip whitespace characters
                    if(len(pkt) == 0):
                        continue # Ignore empty data
                    try:
                        packet_in = json.loads(pkt)
                        
                        # self.__log.console_log(str(packet_in))
                        if type(packet_in) == int:
                            self.__log.console_log("Read int? Discarding")
                            continue
                        if type(packet_in) == float:
                            self.__log.console_log("Read float? Discarding")
                            continue

                        if type(packet_in) != dict:
                            print("Read non dict?" + str(pkt) + " type " + str(type(packet_in)))

                        # print(packet_in)
                        if not self.__rf_set:
                            # Wait for freq change and periodically send new command to try to change freq.
                            if packet_in['type'] == "freq_success":

                                if float(packet_in['frequency']) == float(self.__rf_freq):
                                    self.__log.console_log("Frequency set: Listening on " + str(self.__rf_freq))
                                    self.__rf_set = True
                                else:
                                    self.__log.console_log("Recieved incorrect frequency!")
                                    continue    
                            if packet_in['type'] == "command_success":
                                print("Successful command")
                            else:
                                self.__log.console_log("Recieved packet from wrong stream.. Discarding due to freq change.")
                                continue
                        
                        # No more logging heartbeats

                        if packet_in['type'] == 'data':
                            self.__log.console_log("reading packet type: " + ("sustainer" if packet_in['value']['is_sustainer'] else "booster"))

                    except json.decoder.JSONDecodeError as json_err:
                        self.__log.console_log(f"Recieved corrupted JSON packet. Flushing buffer.")
                        self.__log.console_log(f" ---> DUMP_ERR: Recieved invalid packet of len {len(pkt)} : ")
                        self.__log.fail()
                        self.__log.waiting_delta(-1)
                        continue

                    # Process and queue the packet
                    processed = self.process_packet(packet_in)

                    for combiner in self.__combiners:
                        combiner.enqueue_packet(processed)

                    # Log all packets
                    proc_json = json.dumps(processed)
                    self.__log.file_log(proc_json)
                    self.__log.console_log(f"Processed packet @ {processed['unix']}")
                    self.__log.waiting_delta(-1)
                    self.__log.success()

                    
                    
            except Exception as e:
                try:
                    print(f"[Telem {self.__comport.name}] Ran into an uncaught exception.. continuing gracefully.") # Always print these.
                    self.__log.console_log(f"Error dump:", traceback.format_exc(e))
                except Exception as e:
                    print("Exception while handling excpetion!")

class MQTTThread(threading.Thread):
    """A thread to handle all MQTT communication for the GSS combiner service."""
    def __init__(self, server_uri, log_stream: util.logger.LoggerStream) -> None:
        super(MQTTThread, self).__init__(daemon=True)
        self.__log = log_stream
        self.__uri = server_uri
        self.__mqttclient: mqtt.Client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

        self.__log.console_log("Connecting to broker @ " + str(self.__uri))
        self.__mqttclient.connect(self.__uri, port=1884)
        self.__log.console_log("Subscribing to control streams...")

        self.__booster_cmds = []
        self.__sustainer_cmds = []

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
                self.__log.console_log(f"Failed to decode Command stream packet: {payload_str}")

            self.__log.console_log(f"Recieved command '{raw_cmd}', acknowledged & sent to telemetry threads.")
            ack_msg = {"type": "acknowledge_combiner", "cmd_ack": raw_cmd}
            if(msg.topic == "Control-Sustainer"):
                self.__sustainer_cmds.append(raw_cmd)
                self.__mqttclient.publish(msg.topic, json.dumps(ack_msg))
                return
            if(msg.topic == "Control-Booster"):
                self.__booster_cmds.append(raw_cmd)
                self.__mqttclient.publish(msg.topic, json.dumps(ack_msg))
                return
            

        self.__mqttclient.on_message = on_message
        
        self.__log.console_log("MQTT systems initialized.")

    def get_telem_cmds(self):
        """Returns arrays of queued commands corresponding to (Sustainer, Booster)"""
        sus_cmds = copy.deepcopy(self.__sustainer_cmds)
        boost_cmds = copy.deepcopy(self.__booster_cmds)
        self.__sustainer_cmds = []
        self.__booster_cmds = []
        
        return (sus_cmds, boost_cmds)

    def subscribe_control(self, combiner):
        """Subscribe to a combiner's `control` topic"""
        topic = combiner.get_mqtt_control_topic()
        self.__mqttclient.subscribe(topic) 
        self.__log.console_log("Subscribed to control stream " + str(topic))

    def publish(self, packet_list, topic) -> None:
        """Publish a set of packets to a `Data` topic"""
        self.__log.waiting_delta(len(packet_list))
        for data in packet_list:
            data_encoded = json.dumps(data).encode("utf-8")
            
            self.__log.console_log(f"Publishing {getsizeof(data_encoded)} bytes to MQTT stream --> '{topic}'")

            try:
                self.__mqttclient.publish(topic, data_encoded)
                self.__log.success()

            except Exception as e:
                self.__log.console_log("Unresolved packet dump: ", data_encoded)
                print(f"Unable to publish to '{topic}' : ", str(e)) # Always print
                self.__log.fail()
            self.__log.waiting_delta(-1)

    def run(self) -> None:
        self.__mqttclient.loop_start()

        while True:
            pass
    
    def publish_common(self, data: str):
        """Publish one arbitrary packet to the `Common` topic"""
        self.publish([data], "Common")
