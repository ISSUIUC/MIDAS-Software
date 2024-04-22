from enum import Enum
import datetime

class LoggerOptions():
    def __init__(self, should_log, is_verbose) -> None:
        self.should_log = should_log
        self.is_verbose = is_verbose

class LoggerType(str, Enum):
    TELEM = "TELE"
    COMBINER = "COMB"
    MQTT = "MQTT"

class LoggerStream():
    def __init__(self, options: LoggerOptions, type: LoggerType, name: str) -> None:
        self.__title = type.value + " " + name
        self.__filename = "./outputs/" + str(int(datetime.datetime.now().timestamp())) + "_" + type.value + name + "_raw_output.txt"
        self.__lstype = type
        self.__opts = options
        self.__file = None
        self.__failures = 0
        self.__success = 0
        self.__waiting = 0

        self.__last_result = datetime.datetime.now().timestamp()

        if self.__opts.should_log and type != LoggerType.MQTT:
            self.__file = open(self.__filename, "w")

    def serialize(self) -> dict:
        return {"success": self.__success, "fail": self.__failures, "waiting": self.__waiting}
    
    def get_name(self) -> str:
        return self.__title

    def __print(self, line: str):
        print(f"[{self.__title}] {line}")

    def __log(self, line: str):
        self.__file.write(str(line))

    def get_last_result_time(self):
        return self.__last_result

    def console_log(self, line: str):
        if (self.__opts.is_verbose):
            self.__print(line)

    def file_log(self, data: str):
        if (self.__opts.should_log):
            self.__log(data)

    def success(self):
        self.__success += 1
        self.__last_result = datetime.datetime.now().timestamp()

    def fail(self):
        self.__failures += 1
        self.__last_result = datetime.datetime.now().timestamp()

    def set_waiting(self, number: int):
        self.__waiting = number

    def waiting_delta(self, num):
        self.__waiting += num

    def get_successes(self):
        return self.__success
    
    def get_failures(self):
        return self.__failures
    
    def get_waiting(self):
        return self.__waiting





class Logger():
    def __init__(self, options: LoggerOptions) -> None:
        self.__options = options
        self.__streams = {}
        
    def create_stream(self, stream_type: LoggerType, stream_name: str) -> LoggerStream:
        stream = LoggerStream(self.__options, stream_type, stream_name)
        self.__streams[stream_type + stream_name] = stream
        return stream
    
    def streams(self):
        return self.__streams

def format_stat_string(name, logstream: LoggerStream):
    successes = logstream.get_successes()
    failures = logstream.get_failures()
    waiting = logstream.get_waiting()
    delta_last = datetime.datetime.now().timestamp() - logstream.get_last_result_time()
    total = successes + failures
    pct = 0
    if(total != 0):
        pct = successes / total

    col = "\x1b[32m"

    if(pct < 0.8):
        col = "\x1b[33m"

    if(pct < 0.5):
        col = "\x1b[31m"

    col_waiting = "\x1b[32m"
    if(waiting > 1):
        col_waiting = "\x1b[33m"
    if(waiting > 5):
        col_waiting = "\x1b[31m"

    col_sysname = "\x1b[90m"
    if successes > 0:
        if delta_last < 1:
            col_sysname = "\x1b[32m"
        elif delta_last < 2:
            col_sysname = "\x1b[33m"
        else:
            col_sysname = "\x1b[31m"

    return f"\x1b[1m{col_sysname}{name}\x1b[0m ({col}{(pct*100):.0f}%\x1b[0m : {col_waiting}{waiting}\x1b[0m) " 

def print_legend(uri_target):
    print("Listening on MQTT URI: \x1b[34mmqtt://" + uri_target + ":1883\x1b[0m")
    print("LEGEND:         System   \x1b[1m\x1b[90mInactive   \x1b[32mNominal   \x1b[33mDelayed   \x1b[31mDisconnected\x1b[0m      (Success %  :  Operations Waiting)")
    print()