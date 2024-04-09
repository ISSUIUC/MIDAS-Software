import paho.mqtt.publish as publish
import threading
from collections import deque
import json

packet = json.load(open("./ground/gss_combiner/telem_packet.json"))

class TelemetryThread(threading.Thread):
    def __init__(self, com_port, mqtt_uri, all_data_topic) -> None:
        self.__topic = all_data_topic
        self.__comport = com_port
        self.__uri = mqtt_uri
        self.__queue: deque = deque()

        

    def run(self) -> None:
        while True:
            # read from the comport
            # edit queue / store mac address
            self.__queue.append(packet)
            print(self.__topic, self.__queue.pop(), self.__uri)
            # publish.single(self.__topic, self.__queue.pop(), hostname=self.__uri)
            pass

    def get_queue(self):
        return self.__queue


class MQTTThread(threading.Thread):
    
    def __init__(self, combiners, server_uri) -> None:
        self.__combiners = combiners
        self.__uri = server_uri


    def run(self) -> None:
        while True:
            for combiner in self.combiners__:
                if combiner.empty():
                    continue

                data = combiner.get_best()
                publish.single(combiner.get_mqtt_topic(), data, hostname=self.__uri)
        