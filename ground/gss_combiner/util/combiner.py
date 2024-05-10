# Telemetry Combiner
# ISS Spaceshot Avionics 2024

from datetime import datetime, timezone
from collections import deque
import copy

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
        class DP():
            """A single duplicate packet list checker"""
            def __init__(self, packets) -> None:
                self.__packets = packets
                self.__ts = datetime.now().timestamp()

            def get_ts(self) -> float:
                """Return the timestamp this packet list was added"""
                return self.__ts

            def check(self, packet) -> bool:
                """Returns whether or not this packet should be allowed to be added to the `TelemetryCombiner` (Tests for duplicates)"""
                return True
                try:
                    for pkt in self.__packets:
                        incoming = copy.copy(packet['value'])
                        existing = copy.copy(pkt['value'])
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
            self.__duplicates = []
            self.__timeout = timeout

        def insert(self, pkt_list):
            """Add a packet list to the `DuplicateDatapoints` checker"""
            self.__duplicates.append(TelemetryCombiner.DuplicateDatapoints.DP(pkt_list))

        def clear_old(self):
            """Removes packet lists whose `timeout` has expired."""
            new_list = []
            for dp in self.__duplicates:
                if (datetime.now().timestamp() + self.__timeout > dp.get_ts()):
                    new_list.append(dp)
            self.__duplicates = new_list

        def check(self, packet) -> bool:
            """Returns whether or not this packet should be allowed to be added to the `TelemetryCombiner` (Tests for duplicates)"""
            self.clear_old()
            for dp in self.__duplicates:
                if not dp.check(packet):
                    return False
            return True

            

    # splitter_list is a list of TelemetryCombiners acting as relay recievers.
    def __init__(self, stage, log_stream: util.logger.LoggerStream, filter=FilterOptions()):
        self.__log = log_stream
        self.__stage = stage
        self.__ts_latest = datetime.now(timezone.utc).timestamp()
        self.__filter = filter
        self.__packets_in = deque()
        self.__mqtt_threads = []
        self.__duplicate = TelemetryCombiner.DuplicateDatapoints(2)

    def enqueue_packet(self, packet):
        """Add a packet to this telemetry combiner to be checked and sent"""
        self.__packets_in.append(packet)
        filter = self.filter()
        self.__duplicate.insert(filter)
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
        for packet in queue:
            
            # Check if packet passes filter
            self.__log.console_log("Packet states: flt: (" + str(self.__filter.test(packet)) + ") dup: (" + str(self.__duplicate.check(packet)) + ")")
            if not (packet['unix'] in seen_timestamps) and self.__filter.test(packet) and self.__duplicate.check(packet):
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
    
