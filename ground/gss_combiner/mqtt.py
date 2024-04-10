import paho.mqtt.client as mqtt
import threading
from collections import deque
import json
from datetime import datetime, timezone
import serial # PySerial
from sys import getsizeof


class TelemetryThread(threading.Thread):
    def __init__(self, com_port, mqtt_uri, all_data_topic) -> None:
        super(TelemetryThread, self).__init__(daemon=True)
        self.__topic = all_data_topic
        self.__comport: serial.Serial = serial.Serial(com_port, baudrate=57600)
        self.__uri = mqtt_uri
        self.__queue: deque = deque()
        self.__comport.reset_input_buffer()
        self.__mqttclient = None
        print(f"[Telem {self.__comport.name}] Telemetry thread created.")

        
    def process_packet(packet_json):
        time = datetime.now(timezone.utc)
        return {'value': packet_json, 'type': 'data', 'utc': str(time), 'unix': datetime.timestamp(time)}
    
    def __read_comport(self):
        if self.__comport.in_waiting:
            return self.__comport.read_all()
        else:
            return ""

    def run(self) -> None:

        self.__mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        print(f"[Telem {self.__comport.name}] Connecting to MQTT @ {self.__uri}")
        self.__mqttclient.connect(self.__uri)
        print(f"[Telem {self.__comport.name}] Telemetry thread ready.")

        while True:
            # Wrap all in a try-except to catch errors.
            try:
                # Read all from the comport
                raw_in = self.__read_comport()
                if len(raw_in) == 0:
                    # No data in
                    continue

                packets = bytes.decode(raw_in, encoding='utf-8').split('\n') # Split packets by delimeter
                
                # Process raw to packet

                for pkt in packets:
                    if(len(pkt) == 0):
                        continue # Ignore empty data
                    try:
                        packet_in = json.loads(pkt)
                    except json.decoder.JSONDecodeError as json_err:
                        print(f"[Telem {self.__comport.name}] Recieved corrupted JSON packet. Flushing buffer.")
                        print(f" ---> DUMP_ERR: Recieved invalid packet of len {len(pkt)} : ")
                        continue
                    processed = TelemetryThread.process_packet(packet_in)

                    # Edit queue and send data out.
                    self.__queue.appendleft(processed) # Append left since newer packets will be further down the serial input stream.

                    proc_string = json.dumps(processed).encode('utf-8')
                    self.__mqttclient.publish(self.__topic, proc_string)
                    print(f"[Telem {self.__comport.name}] Processed packet @ {processed['unix']} --> '{self.__topic}'")
            except Exception as e:
                print(f"[Telem {self.__comport.name}] Ran into an uncaught exception.. continuing gracefully.")
                print(f"[Telem {self.__comport.name}] Error dump:", str(e))

    def empty(self):
        return len(self.__queue) == 0

    def get_queue(self):
        return self.__queue
    
    def clear(self):
        self.__queue.clear()


class MQTTThread(threading.Thread):
    
    def __init__(self, combiners, server_uri) -> None:
        super(MQTTThread, self).__init__(daemon=True)
        self.__combiners = combiners
        self.__uri = server_uri
        self.__mqttclient = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

        print("[MQTT] Connecting to broker...")
        self.__mqttclient.connect(self.__uri)
        print("[MQTT] Connected to MQTT")


    def run(self) -> None:
        while True:
            try:
                for combiner in self.__combiners:
                    if combiner.empty():
                        continue

                    data = combiner.get_best()
                    data_encoded = json.dumps(data).encode("utf-8")
                    print(f"[MQTT] Published {getsizeof(data_encoded)} bytes to MQTT stream --> '{combiner.get_mqtt_topic()}'")

                    try:
                        self.__mqttclient.publish(combiner.get_mqtt_topic(), data_encoded)
                    except Exception as e:
                        print("Unresolved packet dump: ", data_encoded)
                        print(f"[MQTT] Unable to publish to '{combiner.get_mqtt_topic()}' : ", str(e))
            except Exception as e:
                print(f"[MQTT] Ran into an uncaught exception.. continuing gracefully.")
                print(f"[MQTT] Error dump: ", str(e))
        