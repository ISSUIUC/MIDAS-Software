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
    def __init__(self, com_port, mqtt_uri, all_data_topic, log_stream: util.logger.LoggerStream) -> None:
        super(TelemetryThread, self).__init__(daemon=True)
        self.__log = log_stream
        self.__topic = all_data_topic
        self.__log.console_log(f"Opening {com_port}")
        self.__comport: serial.Serial = serial.Serial(com_port, baudrate=4800, write_timeout=1)
        self.__uri = mqtt_uri
        self.__comport.reset_input_buffer()
        self.__mqttclient = None
        self.__log.console_log(f"Telemetry thread created.")
        self.__combiners = []

        self.__rf_set = True
        self.__rf_freq = 0
        self.__last_rf_command_sent = datetime.now().timestamp()
        self.__rf_command_period = 3

        
    def process_packet(self, packet_json):
        """Append metadata to a packet to conform to GSS v1.1 packet structure"""
        # Incoming packets are of form {"type": "data", value: { ... }}
        time = datetime.now(timezone.utc)

        # Append timestamps :)
        return {'value': packet_json['value'], 'type': packet_json['type'], 'utc': str(time), 'unix': datetime.timestamp(time), 'src': self.__comport.name}
    
    def write_frequency(self, frequency:float):
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
        
    def run(self) -> None:
        # Initialize MQTT
        self.__mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.__log.console_log(f"Connecting to MQTT @ {self.__uri}")
        self.__mqttclient.connect(self.__uri)
        self.__log.console_log(f"Telemetry thread ready.")

        while True:
            # Wrap all in a try-except to catch errors.

            try:

                if not self.__rf_set and datetime.now().timestamp() - self.__last_rf_command_sent >= self.__rf_command_period:
                    # Send a new frequency command if the last one went unacknowledged.
                    self.__send_comport("FREQ:" + self.__rf_freq + "\n")
                    self.__last_rf_command_sent = datetime.now().timestamp()
                    

                # Read all from the comport
                packets = self.__read_comport()

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

                        # print(packet_in)
                        if not self.__rf_set:
                            # Wait for freq change and periodically send new command to try to change freq.
                            if packet_in['type'] == "freq_success":

                                if float(packet_in['frequency']) == float(self.__rf_freq):
                                    self.__log.console_log_always("Frequency set: Listening on " + str(self.__rf_freq))
                                    self.__rf_set = True
                                else:
                                    self.__log.console_log("Recieved incorrect frequency!")
                                    continue    
                            else:
                                self.__log.console_log("Recieved packet from wrong stream.. Discarding due to freq change.")
                                continue

                        if packet_in['type'] == "heartbeat":
                            # Send heartbeat to common stream
                            heartbeat_only = {"battery_voltage": packet_in['value']['battery_voltage'], 'rssi': packet_in['value']['RSSI']}
                            raw_in = {"source": "gss_combiner", "action": "heartbeat", "time": datetime.now().timestamp(), "data": heartbeat_only}
                            proc_string = json.dumps(raw_in).encode('utf-8')
                            self.__mqttclient.publish("Common", proc_string)
                            self.__log.success()
                            continue

                        if packet_in['type'] == 'data':
                            self.__log.console_log("reading packet type: " + ("sustainer" if packet_in['value']['is_sustainer'] else "booster"))
                        else:
                            # print()
                            # print(packet_in)
                            continue
                    except json.decoder.JSONDecodeError as json_err:
                        self.__log.console_log(f"Recieved corrupted JSON packet. Flushing buffer.")
                        self.__log.console_log(f" ---> DUMP_ERR: Recieved invalid packet of len {len(pkt)} : ")
                        self.__log.fail()
                        self.__log.waiting_delta(-1)
                        continue

                    # Process and queue the packet
                    processed = TelemetryThread.process_packet(packet_in)

                    for combiner in self.__combiners:
                        combiner.enqueue_packet(processed)


                    # Log all packets and send it to the all-data stream
                    proc_json = json.dumps(processed)
                    proc_string = proc_json.encode('utf-8')
                    self.__mqttclient.publish(self.__topic, proc_string)
                    self.__log.file_log(proc_json)
                    self.__log.console_log(f"Processed packet @ {processed['unix']} --> '{self.__topic}'")
                    self.__log.waiting_delta(-1)
                    self.__log.success()
                    
            except Exception as e:
                print(f"[Telem {self.__comport.name}] Ran into an uncaught exception.. continuing gracefully.") # Always print these.
                self.__log.console_log(f"Error dump:", traceback.format_exc(e))


class MQTTThread(threading.Thread):
    """A thread to handle all MQTT communication for the GSS combiner service."""
    def __init__(self, server_uri, log_stream: util.logger.LoggerStream) -> None:
        super(MQTTThread, self).__init__(daemon=True)
        self.__log = log_stream
        self.__uri = server_uri
        self.__mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

        self.__log.console_log("Connecting to broker @ " + str(self.__uri))
        self.__mqttclient.connect(self.__uri)
        self.__log.console_log("Subscribing to control streams...")

        # As of now this system has no need to listen to the control stream.
        # def on_message(client, userdata, msg): 
        #     print(msg.topic+" "+str(msg.payload))

        # self.__mqttclient.on_message = on_message
        
        self.__log.console_log("MQTT systems initialized.")


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
        """Publish arbitrary data to the `Common` topic"""
        try:
            self.__mqttclient.publish("Common", data)
            self.__log.success()
        except Exception as e:
            print(f"Unable to publish to 'Common' : ", str(e)) # Always print
            self.__log.fail()