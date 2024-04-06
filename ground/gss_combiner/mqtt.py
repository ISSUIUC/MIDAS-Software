import paho.mqtt.publish as publish
import threading

class TelemetryThread(threading.Thread):
    def __init__(self, com_port) -> None:
        self.__comport = com_port
        self.__queue = []

    def run(self) -> None:
        while True:
            # read from the comport
            # edit queue / store mac address
            pass

    def get_queue(self):
        return self.__queue
    pass


class MQTTThread(threading.Thread):
    
    def __init__(self, combiners, server_uri, topic) -> None:
        self.__combiners = combiners
        self.__topic = topic
        self.__uri = server_uri


    def run(self) -> None:
        while True:
            for combiner in self.combiners__:
                if combiner.empty():
                    continue

                data = combiner.get_best()
                publish.single(self.__topic, data, hostname=self.__uri)
        

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("$SYS/#")



mqttc.on_connect = on_connect

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
mqttc.loop_forever()