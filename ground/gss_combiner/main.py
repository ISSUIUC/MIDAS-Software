# GSS Combiner. ISS Spaceshot Avionics 2024
# Allows for a centralized telemetry system with a pub/sub architecture.

# USAGE:
# py ./main.py [options]
#  
#    --booster [source1],[source2],[etc..]      -> Selects which COM ports should be interpreted as a data stream from the booster stage
#    --sustainer [source1],[source2],[etc..]    -> Selects which COM ports should be interpreted as a data stream from the sustainer stage
#    --local   (or -l)                          -> Streams all data to 'localhost' for testing. (Same as --ip localhost)
#    --no-log  (or -n)                          -> Will not log data to logfiles for this run
#    --verbose (or -v)                          -> Prints all telemetry events to console
#    --no-vis  (or -nv)                         -> Shows a visual display of all systems
#    --ip [IP] (or -i [IP])                     -> Connects to a specific IP. (Overrides --local)
#    --help    (or -h)                          -> Prints this menu

import sys
import threading
import datetime
import json

import util.mqtt as mqtt
import util.combiner as combiner
import util.logger as logger
import util.print_util


def assert_alive(threads: list[threading.Thread]):
    for thread in threads:
        assert thread.is_alive()

uri_target = "10.195.167.19"

def is_tl_arg(argument):
    return argument.startswith("--") or argument.startswith("-")

def parse_params(arguments):
    num_params = len(arguments)
    arg_ptr = 1 # Skip ./main.py arg

    booster_sources = []
    sustainer_sources = []
    is_local = False
    should_log = True

    has_booster = False
    has_sustainer = False
    is_verbose = False
    is_visual = True

    use_ip = None

    while (arg_ptr < num_params):
        arg = arguments[arg_ptr]
        if(not is_tl_arg(arg)):
            raise ValueError("Invalid argument: " + str(arg))
        
        # Handle booster/sustainer logic
        if (arg == "--booster"):
            if (has_booster):
                raise ValueError("Argument '--booster' is unique (You have defined booster sources twice.)")
            has_booster = True

            next_arg = arguments[arg_ptr + 1]
            if(is_tl_arg(next_arg)):
                raise ValueError("You must pass values to argument " + str(arg) + f"  (got {next_arg})")
            booster_sources = next_arg.split(',')
            arg_ptr += 2
            continue

        if (arg == "--sustainer"):
            if (has_sustainer):
                raise ValueError("Argument '--sustainer' is unique (You have defined sustainer sources twice.)")
            has_sustainer = True

            next_arg = arguments[arg_ptr + 1]
            if(is_tl_arg(next_arg)):
                raise ValueError("You must pass values to argument " + str(arg) + f"  (got {next_arg})")
            sustainer_sources = next_arg.split(',')
            arg_ptr += 2
            continue

        if (arg == "--ip" or arg == "-i"):
            next_arg = arguments[arg_ptr + 1]
            if(is_tl_arg(next_arg)):
                raise ValueError("You must pass values to argument " + str(arg) + f"  (got {next_arg})")
            use_ip = next_arg
            arg_ptr += 2
            continue

        if (arg == "--local" or arg == "-l"):
            is_local = True

        if (arg == "--no-log" or arg == "-n"):
            should_log = False

        if (arg == "--help" or arg == "-h"):
            print(util.print_util.HELP_OUTPUT)
            exit(0)

        if (arg == "--verbose" or arg == "-v"):
            is_verbose = True

        if (arg == "--no-vis" or arg == "-nv"):
            is_visual = False

        arg_ptr += 1


    if not has_sustainer and not has_booster:
        print("\n\x1b[1m\x1b[33mWARNING: No booster or sustainer sources have been selected! You will not read data!\x1b[0m\n")
    
    return booster_sources, sustainer_sources, is_local, should_log, is_verbose, is_visual, use_ip

if __name__ == "__main__":
    threads = []

    booster_sources, sustainer_sources, is_local, should_log, is_verbose, is_visual, ip_override = parse_params(sys.argv)

    if is_local:
        uri_target = "localhost"

    if ip_override is not None:
        uri_target = ip_override

    log = logger.Logger(logger.LoggerOptions(should_log, is_verbose))

    print("Using sustainer sources: ", sustainer_sources)
    print("Using booster sources: ", booster_sources)

    telem_threads_booster = []
    telem_threads_sustainer = []

    for port in booster_sources:
        new_thread = mqtt.TelemetryThread(port, uri_target, "FlightData-All", log.create_stream(logger.LoggerType.TELEM, port))
        telem_threads_booster.append(new_thread)

    for port in sustainer_sources:
        new_thread = mqtt.TelemetryThread(port, uri_target, "FlightData-All", log.create_stream(logger.LoggerType.TELEM, port))
        telem_threads_sustainer.append(new_thread)


    for thd in telem_threads_booster:
        thd.start()

    for thd in telem_threads_sustainer:
        thd.start()

    combiner_sus = combiner.TelemetryCombiner("Sustainer", telem_threads_sustainer, log.create_stream(logger.LoggerType.COMBINER, "Sustainer"))
    combiner_boo = combiner.TelemetryCombiner("Booster", telem_threads_booster, log.create_stream(logger.LoggerType.COMBINER, "Booster"))


    broadcast_thread = mqtt.MQTTThread([combiner_sus, combiner_boo], uri_target, log.create_stream(logger.LoggerType.MQTT, "main"))
    broadcast_thread.start()

    threads = [broadcast_thread] + telem_threads_booster + telem_threads_sustainer

    assert_alive(threads)

    print("\nTelemetry system initialized successfully!\n\n")


    print_delay = 0.5
    last_print_db = datetime.datetime.now().timestamp() + print_delay

    if (not is_verbose and is_visual):
        logger.print_legend(uri_target)

    while True:
        assert_alive(threads)

        if (is_verbose or not is_visual):
            continue

        if(datetime.datetime.now().timestamp() - last_print_db > 0):
            last_print_db = datetime.datetime.now().timestamp() + print_delay

            # Print status
            status_text = ""
            raw_data = {}
            for ls_name, log_stream in log.streams().items():
                log_stream: logger.LoggerStream = log_stream
                status_text += logger.format_stat_string(ls_name, log_stream)
                raw_data[log_stream.get_name()] = log_stream.serialize()
                
            print(f"Status: {status_text}", end="\r")

            # Send status
            send_data = {"source": "gss_combiner", "time": datetime.datetime.now().timestamp(), "data": raw_data}
            broadcast_thread.publish_common(json.dumps(send_data))


# com0com setup:
# COM1 <-> COM16
# COM2 <-> COM17
# COM18 <-> COM19
# COM20 <-> COM21
# & C:/Python311/python.exe c:/Users/mpkar/Documents/ISS/MIDAS-Software/ground/gss_combiner/main.py --sustainer COM1,COM2 --booster COM18,COM20
