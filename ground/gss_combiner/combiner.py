import threading
import serial
import serial.tools.list_ports
import paho.mqtt.publish as publish

import mqtt


class TelemetryCombiner(threading.Thread):
    
    def __init__(self, mqtt_topic, thread_list: mqtt.TelemetryThread):
        self.__threads = thread_list
        self.__mqtt_topic = mqtt_topic

    def empty(self) -> bool:
        return True # TODO
    
    def get_mqtt_topic(self) -> str:
        return self.__mqtt_topic

    def get_best(self):
        for thread in self.threads__:
            pass

        return None