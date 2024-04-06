import threading
import serial
import serial.tools.list_ports

import gss_combiner.mqtt as mqtt


class TelemetryCombiner(threading.Thread):
    def __init__(self, thread_list: mqtt.TelemetryThread):
        self.__threads = thread_list 

    def empty(self) -> bool:
        return True # TODO

    def get_best(self):
        for thread in self.threads__:
            pass

        return None