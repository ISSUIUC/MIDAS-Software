from datetime import datetime, timezone
from sys import getsizeof
from collections import deque
import threading
import json
import traceback

import serial # PySerial
import paho.mqtt.client as mqtt

import util.logger


class TelemetryThread(threading.Thread):
    def __init__(self, com_port, mqtt_uri, all_data_topic, log_stream: util.logger.LoggerStream) -> None:
        super(TelemetryThread, self).__init__(daemon=True)
        self.__log = log_stream
        self.__topic = all_data_topic
        self.__log.console_log(f"Opening {com_port}")
        self.__comport: serial.Serial = serial.Serial(com_port, baudrate=4800)
        self.__uri = mqtt_uri
        self.__queue: deque = deque()
        self.__comport.reset_input_buffer()
        self.__mqttclient = None
        self.__log.console_log(f"Telemetry thread created.")

        
    def process_packet(packet_json):
        # Incoming packets are of form {"type": "data", value: { ... }}
        time = datetime.now(timezone.utc)

        # Append timestamps :)
        return {'value': packet_json['value'], 'type': packet_json['type'], 'utc': str(time), 'unix': datetime.timestamp(time)}
    
    def __read_comport(self):
        if self.__comport.in_waiting:
            return self.__comport.read_all()
        else:
            return ""

    def run(self) -> None:

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
                    # No data in
                    continue

                packets = bytes.decode(raw_in, encoding='utf-8').split('\n')[:-1] # Split packets by delimeter, get newest packets first.

                # Process raw to packet
                self.__log.console_log(f"Processing {len(packets)} packets..")
                self.__log.set_waiting(len(packets))
                for pkt in packets:
                    if(len(pkt) == 0):
                        continue # Ignore empty data
                    try:
                        packet_in = json.loads(pkt)
                    except json.decoder.JSONDecodeError as json_err:
                        self.__log.console_log(f"Recieved corrupted JSON packet. Flushing buffer.")
                        self.__log.console_log(f" ---> DUMP_ERR: Recieved invalid packet of len {len(pkt)} : ")
                        self.__log.fail()
                        self.__log.waiting_delta(-1)
                        continue

                    processed = TelemetryThread.process_packet(packet_in)

                    # Edit queue and send data out.
                    self.__queue.append(processed) # Append left since newer packets will be further down the serial input stream.

                    proc_string = json.dumps(processed).encode('utf-8')
                    self.__mqttclient.publish(self.__topic, proc_string)
                    self.__log.console_log(f"Processed packet @ {processed['unix']} --> '{self.__topic}'")
                    self.__log.waiting_delta(-1)
                    self.__log.success()
                    


            except Exception as e:
                print(f"[Telem {self.__comport.name}] Ran into an uncaught exception.. continuing gracefully.") # Always print these.
                self.__log.console_log(f"Error dump:", traceback.format_exc(e))

    def empty(self):
        return len(self.__queue) == 0

    def get_queue(self):
        return self.__queue
    
    def clear(self):
        self.__queue.clear()


class MQTTThread(threading.Thread):
    
    def __init__(self, combiners, server_uri, log_stream: util.logger.LoggerStream) -> None:
        super(MQTTThread, self).__init__(daemon=True)
        self.__log = log_stream
        self.__combiners = combiners
        self.__uri = server_uri
        self.__mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

        self.__log.console_log("Connecting to broker @ " + str(self.__uri))
        self.__mqttclient.connect(self.__uri)
        self.__log.console_log("Subscribing to control streams...")

        def on_message(client, userdata, msg):
            self.__log.console_log(msg.topic+" "+str(msg.payload))

        self.__mqttclient.on_message = on_message
        

        for combiner in self.__combiners:
            topic = combiner.get_mqtt_control_topic()
            self.__mqttclient.subscribe(topic) 
            self.__log.console_log("Subscribed to control stream " + str(topic))

        self.__log.console_log("MQTT systems initialized.")


    def run(self) -> None:
        self.__mqttclient.loop_start()
        while True:
            
            try:
                for combiner in self.__combiners:
                    
                    if combiner.empty():
                        continue

                    packets = combiner.get_best()
                    self.__log.set_waiting(len(packets))
                    for data in packets:
                        
                        data_encoded = json.dumps(data).encode("utf-8")
                        
                        self.__log.console_log(f"Publishing {getsizeof(data_encoded)} bytes to MQTT stream --> '{combiner.get_mqtt_data_topic()}'")



                        try:
                            self.__mqttclient.publish(combiner.get_mqtt_data_topic(), data_encoded)
                            self.__log.success()
            
                        except Exception as e:
                            self.__log.console_log("Unresolved packet dump: ", data_encoded)
                            print(f"Unable to publish to '{combiner.get_mqtt_data_topic()}' : ", str(e)) # Always print
                            self.__log.fail()
                        self.__log.waiting_delta(-1)
            except Exception as e:
                print(f"Ran into an uncaught exception.. continuing gracefully.") # Always print
                self.__log.console_log(f"Error dump: ", str(e))
        