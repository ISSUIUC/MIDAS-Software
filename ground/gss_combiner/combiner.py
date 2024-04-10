import threading
import serial
import serial.tools.list_ports
import paho.mqtt.publish as publish

import mqtt


class TelemetryCombiner():
    
    def __init__(self, stage, thread_list: mqtt.TelemetryThread):
        self.__threads = thread_list
        self.__stage = stage

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
        best_packet = None
        for thread in self.__threads:
            if not thread.empty():
                queue = thread.get_queue()

                if best_packet is None:
                    best_packet = queue[0]

                if queue[0]['unix'] > best_packet['unix']:
                    best_packet = queue[0]
                    
                
                thread.clear()
    
        return best_packet