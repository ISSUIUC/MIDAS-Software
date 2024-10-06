# GSS Combiner. ISS Spaceshot Avionics 2024
# A pub-sub architecture telemetry system to allow for rapid development of telemetry-consuming systems
# -----------------------------------------------------------------------------------------------------
# Peter Giannetos (2025)
# Aidan Costello (2026)
# Aaditya Voruganti (2026)
# Zyun Lam (2027)
# Michael Karpov (2027)

# USAGE:
# py ./main.py [options]
#  
#    --booster [source1],[source2],[etc..]      -> Selects which COM ports should be interpreted as a data stream from the booster stage
#    --sustainer [source1],[source2],[etc..]    -> Selects which COM ports should be interpreted as a data stream from the sustainer stage
#    --relay [source1],[source2],[etc..]        -> Selects which COM ports should be interpreted as a data stream from the telemetry relay
#    --local   (or -l)                          -> Streams all data to 'localhost' for testing. (Same as --ip localhost)
#    --no-log  (or -n)                          -> Will not log data to logfiles for this run
#    --verbose (or -v)                          -> Prints all telemetry events to console
#    --no-vis  (or -nv)                         -> Shows a visual display of all systems
#    --ip [IP] (or -i [IP])                     -> Connects to a specific IP. (Overrides --local)
#    --help    (or -h)                          -> Prints this menu
#    --config [config] (or -c [config])         -> Uses an argument config defined in config.ini. Added on top of existing params.
#    --no-rf                                    -> Does not overwrite feather frequencies on startup

import sys
import threading
import datetime
import json
import configparser

import util.mqtt as mqtt
import util.combiner as combiner
import util.logger as logger
import util.print_util


uri_target = ""
cfg = configparser.ConfigParser()
cfg.read("./config.ini")

def assert_alive(threads: list[threading.Thread]):
    """Used in the main loop of the combiner to ensure that all threads stay alive."""
    for thread in threads:
        assert thread.is_alive()

def threads_ready(threads: list[mqtt.TelemetryThread]):
    """Used to ensure telemetry threads are ready"""
    for thread in threads:
        if not thread.ready():
            return False
    return True

def parse_params(arguments):
    """Parse parameters passed into this script and return them as a flat tuple."""

    def is_tl_arg(argument):
        """Helper function for determining whether a cli argument is a top-level argument or not.
        Returns true if the argument is a top-level argument."""
        return argument.startswith("--") or argument.startswith("-")

    num_params = len(arguments)
    arg_ptr = 1 # Skip ./main.py arg

    booster_sources = []
    sustainer_sources = []
    relay_sources = []
    is_local = False
    should_log = True

    has_booster = False
    has_sustainer = False
    has_relay = False
    is_verbose = False
    is_visual = True

    use_ip = None
    use_config = None
    overwrite_rf = True

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

        if (arg == "--relay"):
            if (has_relay):
                raise ValueError("Argument '--relay' is unique (You have defined relay sources twice.)")
            has_relay = True

            next_arg = arguments[arg_ptr + 1]
            if(is_tl_arg(next_arg)):
                raise ValueError("You must pass values to argument " + str(arg) + f"  (got {next_arg})")
            relay_sources = next_arg.split(',')
            arg_ptr += 2
            continue

        # Handle misc arguments
        if (arg == "--ip" or arg == "-i"):
            next_arg = arguments[arg_ptr + 1]
            if(is_tl_arg(next_arg)):
                raise ValueError("You must pass values to argument " + str(arg) + f"  (got {next_arg})")
            use_ip = next_arg
            arg_ptr += 2
            continue

        if (arg == "--config" or arg == "-c"):
            next_arg = arguments[arg_ptr + 1]
            if(is_tl_arg(next_arg)):
                raise ValueError("You must pass values to argument " + str(arg) + f"  (got {next_arg})")
            use_config = next_arg
            arg_ptr += 2
            continue

        if (arg == "--local" or arg == "-l"):
            is_local = True

        if (arg == "--no-log" or arg == "-n"):
            should_log = False

        if (arg == "--no-rf"):
            overwrite_rf = False

        if (arg == "--help" or arg == "-h"):
            print(util.print_util.HELP_OUTPUT)
            exit(0)

        if (arg == "--verbose" or arg == "-v"):
            is_verbose = True

        if (arg == "--no-vis" or arg == "-nv"):
            is_visual = False

        arg_ptr += 1


    
    return booster_sources, sustainer_sources, relay_sources, is_local, should_log, is_verbose, is_visual, use_ip, overwrite_rf, use_config

if __name__ == "__main__":
    SCRIPT_START_TIME = datetime.datetime.now().timestamp()
    threads = []

    booster_sources, sustainer_sources, relay_sources, is_local, should_log, is_verbose, is_visual, ip_override, overwrite_rf, use_config = parse_params(sys.argv)

    if(use_config is not None):
        # If a config is specified, re-interpret parameters once using the config.
        try:
            CFG_ARGS = cfg[use_config]['args'].split(" ")
        except:
            print("Argument config does not exist!")
            exit(1)

        print("Using config ", use_config)
        print("Using command", " ".join(sys.argv + CFG_ARGS) + "\n")
        booster_sources, sustainer_sources, relay_sources, is_local, should_log, is_verbose, is_visual, ip_override, overwrite_rf, use_config = parse_params(sys.argv + CFG_ARGS)
    
    # Ensure that the user is notified if they do not specify any data sources.
    if len(booster_sources)==0 and len(sustainer_sources)==0 and len(relay_sources)==0:
        print("\n\x1b[1m\x1b[33mWARNING: No sources have been selected! You will not read data!\x1b[0m\n")

    if is_local:
        uri_target = "localhost"

    if ip_override is not None:
        uri_target = ip_override

    log = logger.Logger(logger.LoggerOptions(should_log, is_verbose))

    print("Using sustainer sources: ", sustainer_sources)
    print("Using booster sources: ", booster_sources)
    print("Using relay sources: ", relay_sources, "\n\n")

    telem_threads_booster = []
    telem_threads_sustainer = []
    telem_threads_relay = []

    # Initialize primary MQTT thread
    broadcast_thread = mqtt.MQTTThread(uri_target, log.create_stream(logger.LoggerType.MQTT, "main", "mqtt"))    

    # Initialize telemetry combiners
    combiner_sustainer = combiner.TelemetryCombiner("Sustainer", log.create_stream(logger.LoggerType.COMBINER, "Sustainer", "sustainer_comb"), filter=combiner.TelemetryCombiner.FilterOptions(allow_sustainer=True))
    combiner_booster = combiner.TelemetryCombiner("Booster", log.create_stream(logger.LoggerType.COMBINER, "Booster", "booster_comb"), filter=combiner.TelemetryCombiner.FilterOptions(allow_booster=True))

    # Set up MQTT and control streams
    combiner_sustainer.add_mqtt(broadcast_thread)
    combiner_booster.add_mqtt(broadcast_thread)
    broadcast_thread.subscribe_control(combiner_booster)
    broadcast_thread.subscribe_control(combiner_sustainer)

    # Determine RF frequencies from config
    FREQ_SUSTAINER = cfg['config:rf']['rfSustainer']
    FREQ_BOOSTER = cfg['config:rf']['rfBooster']
    FREQ_RELAY = cfg['config:rf']['rfRelay']

    # IMPORTANT! --------------------------------------------------------------------------------------------------------------------------------------
    # When decoding packets for telemetry purposes, we encode callsign in a 'callsign bit' which determines which callsign is being used for that stage
    # When this bit is set, the callsign is interpreted as KD9ZMJ
    # When this bit is NOT set, the callsign is interpreted as KD9ZPM
    # -------------------------------------------------------------------------------------------------------------------------------------------------

    if overwrite_rf:
        print("Waiting for telemetry thread RF initialization... (This may take a bit)")
    else:
        print("Skipping Frequency override")

    # Set up booster telemetry threads
    for port in booster_sources:
        new_thread = mqtt.TelemetryThread(port, uri_target, "FlightData-All", log.create_stream(logger.LoggerType.TELEM, port, "booster_telem"))
        new_thread.add_combiner(combiner_booster)
        if overwrite_rf:
            new_thread.write_frequency(FREQ_BOOSTER)
        telem_threads_booster.append(new_thread)

    # Set up sustainer telemetry threads
    for port in sustainer_sources:
        new_thread = mqtt.TelemetryThread(port, uri_target, "FlightData-All", log.create_stream(logger.LoggerType.TELEM, port, "sustainer_telem"))
        new_thread.add_combiner(combiner_sustainer)
        if overwrite_rf:
            new_thread.write_frequency(FREQ_SUSTAINER)
        telem_threads_sustainer.append(new_thread)

    # Set up telemetry threads for drone relay
    for port in relay_sources:
        new_thread = mqtt.TelemetryThread(port, uri_target, "FlightData-All", log.create_stream(logger.LoggerType.TELEM, port, "relay_telem"))
        new_thread.add_combiner(combiner_booster)
        new_thread.add_combiner(combiner_sustainer)
        if overwrite_rf:
            new_thread.write_frequency(FREQ_RELAY)
        telem_threads_relay.append(new_thread)


    # Start all threads
    for thd in telem_threads_booster:
        thd.start()

    for thd in telem_threads_sustainer:
        thd.start()

    for thd in telem_threads_relay:
        thd.start()

    broadcast_thread.start()
    threads = [broadcast_thread] + telem_threads_booster + telem_threads_sustainer
    assert_alive(threads) # Ensure all threads initialized successfully

    # Set up visualization variables
    print_delay = 0.5
    last_print_db = datetime.datetime.now().timestamp() + print_delay

    # Wait for telem threads to be ready
    init_time_warned = False
    while True:
        if threads_ready(telem_threads_booster + telem_threads_relay + telem_threads_sustainer):
            break

        # Inform user of possible errors if init takes too long..
        time_delta = datetime.datetime.now().timestamp() - SCRIPT_START_TIME
        if time_delta > 20 and not init_time_warned:
            print("\x1b[33mWARNING: RF initialization is taking longer than expected... Make sure that the \x1b[36mconfig.ini\x1b[33m RF configuration is within Feather range.\x1b[0m")
            init_time_warned = True


    print("\n\n\nTelemetry system initialized successfully!\n\n")

    # Print visualization legend
    if (not is_verbose and is_visual):
        logger.print_legend(uri_target)


    while True:
        # Main loop:
        # Ensures threads stay alive and displays visualization of system health

        assert_alive(threads)

        if (is_verbose or not is_visual):
            continue

        # Only print occasionally to not flood standard print
        if(datetime.datetime.now().timestamp() - last_print_db > 0):
            last_print_db = datetime.datetime.now().timestamp() + print_delay

            # Print status
            status_text = ""
            raw_data = {}
            for ls_name, log_stream in log.streams().items():
                log_stream: logger.LoggerStream = log_stream
                status_text += logger.format_stat_string(ls_name, log_stream)
                meta_cat, data = log_stream.serialize()

                if not (meta_cat in raw_data):
                    raw_data[meta_cat] = {}
                     
                raw_data[meta_cat][log_stream.get_name()] = data
                
            print(f"Status: {status_text}", end="\r")

            # Send status
            send_data = {"source": "gss_combiner", "action": "none", "time": datetime.datetime.now().timestamp(), "data": raw_data}
            broadcast_thread.publish_common(json.dumps(send_data))