from datetime import datetime, timezone
from sys import getsizeof
import threading
import json
import traceback
import copy

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

        
    def process_packet(packet_json):
        """Append metadata to a packet to conform to GSS v1.1 packet structure"""
        # Incoming packets are of form {"type": "data", value: { ... }}
        time = datetime.now(timezone.utc)

        # Append timestamps :)
        return {'value': packet_json['value'], 'type': packet_json['type'], 'utc': str(time), 'unix': datetime.timestamp(time)}
    
    def write_frequency(self, frequency:float):
        """Send a FREQ command to the associated telemetry device to change frequencies"""
        self.__log.console_log("Writing frequency command (FREQ:" + str(frequency) + ")")
        try:
            self.__send_comport("FREQ:" + str(frequency))
        except Exception as e:
            print("Unable to send FREQ command: ", e, "Continuing with no FREQ change.")

    def __send_comport(self, msg: str):
        """Send arbirtary data to the associated COM port"""
        self.__comport.write(msg.encode())

    def __read_comport(self):
        """Read all data from the associated COM port"""
        if self.__comport.in_waiting:
            return self.__comport.read_all()
        else:
            return ""
        
    def add_combiner(self, combiner):
        """Add a combiner data sink for this telemetry thread"""
        self.__combiners.append(combiner)
        
    def run(self) -> None:
        # Initialize MQTT
        self.__mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.__log.console_log(f"Connecting to MQTT @ {self.__uri}")
        self.__mqttclient.connect(self.__uri)
        self.__log.console_log(f"Telemetry thread ready.")

        while True:
            # Wrap all in a try-except to catch errors.
            try:
                # Read all from the comport
                raw_in = self.__read_comport()

                if len(raw_in) == 0:
                    # Defer if no data in COM port
                    continue

                packets = bytes.decode(raw_in, encoding='utf-8').split('\n')[:-1] # Split packets by delimeter, get newest packets first.
                
                # Process raw to packet
                self.__log.console_log(f"Processing {len(packets)} packets..")
                self.__log.set_waiting(len(packets))
                for pkt_r in packets:
                    pkt = pkt_r.rstrip() # Strip whitespace characters

                    if(len(pkt) == 0):
                        continue # Ignore empty data
                    try:
                        packet_in = json.loads(pkt)
                        if packet_in['type'] == 'data':
                            self.__log.console_log("reading packet type: " + ("sustainer" if packet_in['value']['is_sustainer'] else "booster"))
                        else:
                            print(packet_in)
                            continue
                    except json.decoder.JSONDecodeError as json_err:
                        print()
                        print(json_err)
                        print(pkt)
                        print()
                        print(raw_in)
                        print()
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