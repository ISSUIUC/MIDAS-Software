# GSS v1.1 logger utility
# Spaceshot Avionics Software 2024

from enum import Enum
import datetime

class LoggerOptions():
    def __init__(self, should_log, is_verbose) -> None:
        self.should_log = should_log
        self.is_verbose = is_verbose

class LoggerType(str, Enum):
    """Enum for possible log types"""
    TELEM = "TELE"
    COMBINER = "COMB"
    MQTT = "MQTT"

class LoggerStream():
    """Class to encapsulate logging at different levels for the GSS combiner system"""
    def __init__(self, options: LoggerOptions, type: LoggerType, name: str, meta_category:str) -> None:
        self.__title = type.value + " " + name
        self.__filename = "./outputs/" + str(int(datetime.datetime.now().timestamp())) + "_" + type.value + name + "_raw_output.txt"
        self.__lstype = type
        self.__meta_cat = meta_category
        self.__opts = options
        self.__file = None
        self.__failures = 0
        self.__success = 0
        self.__waiting = 0

        self.__last_result = datetime.datetime.now().timestamp()

        if self.__opts.should_log and type != LoggerType.MQTT:
            self.__file = open(self.__filename, "w")

    def serialize(self) -> dict:
        """Turn this stream's vitals into an easy-to-send format"""
        return self.__meta_cat, {"success": self.__success, "fail": self.__failures, "waiting": self.__waiting, "last": self.__last_result}
    

    
    def get_name(self) -> str:
        return self.__title

    def __print(self, line: str):
        """internal function to print data"""
        print(f"[{self.__title}] {line}")

    def __log(self, line: str):
        """internal function to log to file"""
        self.__file.write(str(line))

    def get_last_result_time(self):
        """Get the last time this logger has had a `success` or `fail`."""
        return self.__last_result

    def console_log(self, line: str):
        """Log to a console depending on verbosity level"""
        if (self.__opts.is_verbose):
            self.__print(line)

    def console_log_always(self, line: str):
        """Log to a console depending on verbosity level"""
        self.__print(line)

    def file_log(self, data: str):
        """Log to a file depending on whether logging is enabled or not."""
        if (self.__opts.should_log):
            self.__log(data)

    def success(self):
        """Flag a successful transaction (useful for system health)"""
        self.__success += 1
        self.__last_result = datetime.datetime.now().timestamp()

    def fail(self):
        """Flag a failed transaction (useful for system health)"""
        self.__failures += 1
        self.__last_result = datetime.datetime.now().timestamp()

    def set_waiting(self, number: int):
        """Set the amount of waiting operations for this system (useful for system health)"""
        self.__waiting = number

    def waiting_delta(self, num):
        """Change the amount of waiting operations for this system (useful for system health)"""
        self.__waiting += num

    def get_successes(self):
        return self.__success
    
    def get_failures(self):
        return self.__failures
    
    def get_waiting(self):
        return self.__waiting

class Logger():
    """Semi-singleton class for all logger streams in GSS 1.1"""
    def __init__(self, options: LoggerOptions) -> None:
        self.__options = options
        self.__streams = {}
        
    def create_stream(self, stream_type: LoggerType, stream_name: str, stream_meta: str) -> LoggerStream:
        stream = LoggerStream(self.__options, stream_type, stream_name, stream_meta)
        self.__streams[stream_type + stream_name] = stream
        return stream
    
    def streams(self):
        return self.__streams

def format_stat_string(name, logstream: LoggerStream):
    """Format a system's status string depending on the contents of the provided `logstream`"""
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
        if delta_last < 1.5:
            col_sysname = "\x1b[32m"
        elif delta_last < 3:
            col_sysname = "\x1b[33m"
        else:
            col_sysname = "\x1b[31m"

    return f"\x1b[1m{col_sysname}{name}\x1b[0m ({col}{(pct*100):.0f}%\x1b[0m : {col_waiting}{waiting}\x1b[0m) " 

def print_legend(uri_target):
    """Print the legend for the system health display"""
    print("Listening on MQTT URI: \x1b[34mmqtt://" + uri_target + ":1883\x1b[0m")
    print("LEGEND:         System   \x1b[1m\x1b[90mInactive   \x1b[32mNominal   \x1b[33mDelayed   \x1b[31mDisconnected\x1b[0m      (Success %  :  Operations Waiting)")
    print()