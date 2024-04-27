import threading
import serial
import serial.tools.list_ports
import paho.mqtt.publish as publish
from datetime import datetime, timezone

import util.mqtt as mqtt
import util.logger


class TelemetryCombiner():

    class FilterOptions():
        def __init__(self, allow_booster=False, allow_sustainer=False) -> None:
            self.__allow_booster = allow_booster
            self.__allow_sustainer = allow_sustainer

        def test(self, packet) -> bool:
            if (packet['value']['is_sustainer'] and self.__allow_sustainer):
                return True
            
            if (not packet['value']['is_sustainer'] and self.__allow_booster):
                return True
            
            return False
            
            

    # splitter_list is a list of TelemetryCombiners acting as relay recievers.
    def __init__(self, stage, dedicated_thread_list: list[mqtt.TelemetryThread], log_stream: util.logger.LoggerStream, filter=FilterOptions()):
        self.__log = log_stream
        self.__threads = dedicated_thread_list
        self.__stage = stage
        self.__ts_latest = datetime.now(timezone.utc).timestamp()
        self.__filter = filter

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

    def clear_threads(self):
        for thread in self.__threads:
            thread.clear()


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
                    # Check if packet passes filter
                    if not (packet['unix'] in seen_timestamps) and self.__filter.test(packet):
                        # Do use this packet


                        if self.__ts_latest > packet['unix']:
                            self.__log.console_log(f"Released packet out of order! {self.__ts_latest} > {packet['unix']}")

                        if (packet['unix'] > cur_latest):
                            cur_latest = packet['unix']

                        seen_timestamps.add(packet['unix'])
                        packet_release.append(packet)
                        
                        self.__log.success()
                        self.__log.waiting_delta(-1)

                
        self.__ts_latest = cur_latest
        self.__log.file_log(str(packet_release))
        return packet_release