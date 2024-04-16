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
        self.__ts_latest = datetime.now(timezone.utc).timestamp()

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
        cur_latest = self.__ts_latest
        self.__log.set_waiting(0)
        for thread in self.__threads:
            
            if not thread.empty():
                
                queue = thread.get_queue()
                self.__log.waiting_delta(len(queue))

                if best_packet is None:
                    best_packet = queue[0]

                for packet in queue:
                    if not (packet['unix'] in seen_timestamps):
                        # Do send this packet

                        if self.__ts_latest > packet['unix']:
                            self.__log.console_log(f"Released packet out of order! {self.__ts_latest} > {packet['unix']}")

                        if (packet['unix'] > cur_latest):
                            cur_latest = packet['unix']

                        seen_timestamps.add(packet['unix'])
                        packet_release.append(packet)
                        
                        self.__log.success()
                        self.__log.waiting_delta(-1)

                    

                    
                thread.clear()
        self.__ts_latest = cur_latest
        self.__log.file_log(str(packet_release))
        return packet_release