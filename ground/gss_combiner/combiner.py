import threading
import gss_combiner.mqtt as mqtt

class TelemetryCombiner(threading.Thread):
    def __init__(self, thread_list: mqtt.TelemetryThread):
        self.threads__ = thread_list 


    def get_best(self):
        for thread in self.threads__:
            pass

        return None