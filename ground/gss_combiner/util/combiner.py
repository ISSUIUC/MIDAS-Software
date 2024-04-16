import threading
import serial
import serial.tools.list_ports
import paho.mqtt.publish as publish
from datetime import datetime, timezone

import util.mqtt as mqtt
import util.logger


class TelemetryCombiner():
    
    def __init__(self, stage, thread_list: mqtt.TelemetryThread, log_stream: util.logger.LoggerStream):
        self.__log = log_stream
        self.__threads = thread_list
        self.__stage = stage
        self.__ts_latest = time = datetime.now(timezone.utc).timestamp()

    def empty(self) -> bool:
        for thread in self.__threads:
            # print("Try thread .. ", thread.get_queue())
            if not thread.empty():
                return False
        return True
    
    def get_mqtt_data_topic(self) -> str:
        return "FlightData-" + self.__stage
    
    def get_mqtt_control_topic(self) -> str:
        return "Control-" + self.__stage

    def get_best(self):
        seen_timestamps = set()
        packet_release = []
        best_packet = None
        self.__log.set_waiting(0)
        for thread in self.__threads:
            
            if not thread.empty():
                
                queue = thread.get_queue()
                self.__log.waiting_delta(len(queue))

                if best_packet is None:
                    best_packet = queue[0]

                for packet in queue:
                    if not (packet['unix'] in seen_timestamps):
                        # Do not send this packet
                        seen_timestamps.add(packet['unix'])
                        packet_release.append(packet)
                        self.__log.success()
                        self.__log.waiting_delta(-1)
                    
                thread.clear()
    
        return packet_release