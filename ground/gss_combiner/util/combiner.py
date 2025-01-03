# Telemetry Combiner
# ISS Spaceshot Avionics 2024

from datetime import datetime, timezone
from collections import deque
import copy
import time

import util.mqtt as mqtt
import util.logger

class TelemetryCombiner():
    """A class that combines multiple data streams from an antenna array into a single coherent stream to be interpreted by ISS telemetry systems
    
    Allows for filtering packets based on rocket stages as well as fine control over the handling of duplicate packets"""

    class FilterOptions():
        """A helper filter class to define which packets are allowed to be sent to a `TelemetryCombiner`"""
        def __init__(self, allow_booster=False, allow_sustainer=False) -> None:
            self.__allow_booster = allow_booster
            self.__allow_sustainer = allow_sustainer

        def test(self, packet) -> bool:
            """Returns whether or not this packet should be sent to this `TelemetryCombiner`"""
            if (packet['value']['is_sustainer'] and self.__allow_sustainer):
                return True
            
            if (not packet['value']['is_sustainer'] and self.__allow_booster):
                return True
            
            if(packet['value']['battery_voltage'] >= 15.75):
                return False
            
            if(packet['value']['battery_voltage'] <= 0.1):
                return False
            
            return False
        
    class DuplicateDatapoints():
        """Helper class to handle duplicate handling in `TelemetryCombiner`"""

        """Holds each unique packet for the duplicate checker to test off of"""
        class UniquePacket():
            def __init__(self, packet):
                self.__packet = packet
                self.__ts = datetime.now().timestamp()
            
            def get_ts(self) -> float:
                """Return the timestamp this packet list was added"""
                return self.__ts

            def check(self, packet) -> bool:
                """Returns whether or not this packet should be allowed to be added to the `TelemetryCombiner` based on the current UniquePacket (Tests for duplicates)"""
                try:
                    incoming = copy.copy(packet['data'])
                    existing = copy.copy(self.__packet['data'])

                    incoming['RSSI'] = 0 # This will compare everything but RSSI, filter out identical packets with different RSSIs.
                    existing['RSSI'] = 0

                    # print(incoming, existing)
                    if incoming == existing:
                        return False
                    
                    return True
                except Exception as e:
                    print(e)
                    print("Unable to detect duplicates with packet.")
                    return True
                

        def __init__(self, timeout: float) -> None:
            self.__timeout = timeout
            self.__unique_packets = []

        def insert(self, packet):
            """Add a packet list to the `DuplicateDatapoints` checker"""
            self.__unique_packets.append(TelemetryCombiner.DuplicateDatapoints.UniquePacket(packet))

        def clear_old(self):
            """Removes packet lists whose `timeout` has expired."""
            new_list = []
            for up in self.__unique_packets:
                if (datetime.now().timestamp() + self.__timeout > up.get_ts()):
                    new_list.append(up)
            self.__unique_packets = new_list

        def check(self, packet) -> bool:
            """Returns whether or not this packet should be allowed to be added to the `TelemetryCombiner` (Tests for duplicates)"""
            self.clear_old()
            for pkt in self.__unique_packets:
                if not pkt.check(packet):
                    return False
            return True

            

    # splitter_list is a list of TelemetryCombiners acting as relay recievers.
    def __init__(self, stage, log_stream: util.logger.LoggerStream, filter=None):
        if filter is None:
            filter = TelemetryCombiner.FilterOptions()
            
        self.__log = log_stream
        self.__stage = stage
        self.__ts_latest = datetime.now(timezone.utc).timestamp()
        self.__filter = filter
        self.__packets_in = deque()
        self.__mqtt_threads: list[mqtt.MQTTThread] = []
        self.__duplicate = TelemetryCombiner.DuplicateDatapoints(2)

    def enqueue_packet(self, packet):
        """Add a packet to this telemetry combiner to be checked and sent"""

        # Add packet metadata
        packet_new = {
            "data": packet,
            "metadata": {
                "raw_stream": self.get_mqtt_data_topic(),
                "time_published": time.time()
            }
        }

        self.__packets_in.append(packet_new)

        
        filter, used = self.filter()
        
        if used:
            self.__duplicate.insert(packet_new)

        for mqtt_src in self.__mqtt_threads:
            mqtt_src.publish(filter, self.get_mqtt_data_topic())

    def add_mqtt(self, mqtt_thread: mqtt.MQTTThread):
        """Add an MQTT data sink to this combiner"""
        self.__mqtt_threads.append(mqtt_thread)

    def empty(self) -> bool:
        """Returns whether or not this combiner is empty (has no waiting packets)"""
        return len(self.__packets_in) == 0
    
    def get_mqtt_data_topic(self) -> str:
        """Return the data topic for the MQTT stream for this combiner"""
        return "FlightData-" + self.__stage
    
    def get_mqtt_control_topic(self) -> str:
        """Return the control topic for the MQTT stream for this combiner"""
        return "Control-" + self.__stage

    def clear(self):
        """Clear the packet queue of this combiner"""
        self.__packets_in.clear()

    def filter(self):
        """Returns a filtered list of packets that should be sent to the system from this combiner"""
        seen_timestamps = set()
        packet_release = []
        cur_latest = self.__ts_latest
        self.__log.set_waiting(len(self.__packets_in))
        queue = copy.copy(self.__packets_in)
        self.__packets_in = deque()

        # flag to tell if the most recently added packet was kept when filtering
        used = False

        for packet in queue:

            packet_test = self.__filter.test(packet['data'])
            dubplicate_test = self.__duplicate.check(packet['data'])
            
            # Check if packet passes filter
            self.__log.console_log("Packet states: flt: (" + str(packet_test) + ") dup: (" + str(dubplicate_test) + ")")
            if not (packet['data']['unix'] in seen_timestamps) and packet_test and dubplicate_test:
                # Use this packet

                
                
                if self.__ts_latest > packet['data']['unix']:
                    self.__log.console_log(f"Released packet out of order! {self.__ts_latest} > {packet['data']['unix']}")

                if (packet['data']['unix'] > cur_latest):
                    cur_latest = packet['data']['unix']

                seen_timestamps.add(packet['data']['unix'])
                packet_release.append(packet)
                
                self.__log.success()
                self.__log.waiting_delta(-1)

                used = True
            else:
                used = False

                
        self.__ts_latest = cur_latest
        self.__log.file_log(str(packet_release))
        return packet_release, used
    
