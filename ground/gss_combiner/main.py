# GSS Combiner. ISS Spaceshot Avionics 2024
# A pub-sub architecture telemetry system to allow for rapid development of telemetry-consuming systems
# -----------------------------------------------------------------------------------------------------
# Peter Giannetos (2025)
# Aidan Costello (2026)
# Aaditya Voruganti (2026)
# Zyun Lam (2027)
# Michael Karpov (2027)
# Surag Nuthulapaty (2027)

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
import argparse

import util.mqtt as mqtt
import util.combiner as combiner
import util.logger as logger
import util.print_util
from serial.tools import list_ports

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
    arg_parser = argparse.ArgumentParser(
        prog='GSS Combiner',
        description="A system to take telemetry from different sources and pass it onto a MQTT broker for ground station visualization"
    )

    arg_parser.add_argument("--booster", type=str, help="Selects which COM ports should be interpreted as a data stream from the booster stage")
    arg_parser.add_argument("--sustainer", type=str, help="Selects which COM ports should be interpreted as a data stream from the sustainer stage")
    arg_parser.add_argument("--relay", type=str, help="Selects which COM ports should be interpreted as a data stream from the telemetry relay")
    
    arg_parser.add_argument("--nboosters", type=int, help="Specifies the number of boosters")
    arg_parser.add_argument("--nsustainers", type=int, help="Specifies the number of sustainers")
    arg_parser.add_argument("--nrelays", type=int, help="Specifies the number of relays")

    arg_parser.add_argument("-n", "--no-log", action="store_true", help="Will not log data to logfiles for this run")
    arg_parser.add_argument("-v", "--verbose", action="store_true", help="Prints all telemetry events to console")
    arg_parser.add_argument("-nv", "--no-vis", action="store_true", help="Shows a visual display of all systems")
    arg_parser.add_argument("-c", "--config", type=str, help="Uses an argument config defined in config.ini. Added on top of existing params.")
    arg_parser.add_argument("--no-rf", action="store_true", help="Does not overwrite feather frequencies on startup")
    
    arg_parser.add_argument("-l", "--local", action="store_true", help="Streams all data to 'localhost' for testing. (Same as --ip localhost)")
    arg_parser.add_argument("-i", "--ip", type=str, help="Connects to a specific IP. (Overrides --local)")
    
    args = arg_parser.parse_args(arguments)

    booster_sources = []
    sustainer_sources = []
    relay_sources = []

    if args.booster is not None:
        booster_sources = args.booster.split(",")

    if args.sustainer is not None:
        sustainer_sources = args.sustainer.split(",")

    if args.relay is not None:
        relay_sources = args.relay.split(",")

    if args.nboosters is not None:
        for i in range(0, args.nboosters):
            booster_sources.append(str(list_ports.comports()[i]).split(' ')[0])

    if args.nsustainers is not None:
        for i in range(args.nboosters, args.nsustainers + args.nboosters):
            sustainer_sources.append(str(list_ports.comports()[i]).split(' ')[0])

    if args.nrelays is not None:
        for i in range(args.nsustainers + args.nboosters, args.nrelays + args.nsustainers + args.nboosters):
            relay_sources.append(str(list_ports.comports()[i]).split(' ')[0])

    is_local = args.local
    should_log = not args.no_log

    is_verbose = args.verbose
    is_visual = not args.no_vis

    use_ip = args.ip
    use_config = args.config
    overwrite_rf = not args.no_rf
    
    return booster_sources, sustainer_sources, relay_sources, is_local, should_log, is_verbose, is_visual, use_ip, overwrite_rf, use_config

if __name__ == "__main__":
    SCRIPT_START_TIME = datetime.datetime.now().timestamp()
    threads = []
    booster_sources, sustainer_sources, relay_sources, is_local, should_log, is_verbose, is_visual, ip_override, overwrite_rf, use_config = parse_params(sys.argv[1:])

    if(use_config is not None):
        # If a config is specified, re-interpret parameters once using the config.
        try:
            CFG_ARGS = cfg[use_config]['args'].split(" ")
        except:
            print("Argument config does not exist!")
            exit(1)

        print("Using config ", use_config)
        print("Using command", " ".join(sys.argv + CFG_ARGS) + "\n")
        booster_sources, sustainer_sources, relay_sources, is_local, should_log, is_verbose, is_visual, ip_override, overwrite_rf, use_config = parse_params(sys.argv[1:] + CFG_ARGS)
    
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

    telem_threads_booster: list[mqtt.TelemetryThread] = []
    telem_threads_sustainer: list[mqtt.TelemetryThread] = []
    telem_threads_relay: list[mqtt.TelemetryThread] = []

    all_telemetry_threads = telem_threads_booster + telem_threads_sustainer + telem_threads_relay

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
        new_thread = mqtt.TelemetryThread(port, log.create_stream(logger.LoggerType.TELEM, port, "booster_telem"))
        new_thread.add_combiner(combiner_booster)
        if overwrite_rf:
            new_thread.write_frequency(FREQ_BOOSTER)
        telem_threads_booster.append(new_thread)

    # Set up sustainer telemetry threads
    for port in sustainer_sources:
        new_thread = mqtt.TelemetryThread(port, log.create_stream(logger.LoggerType.TELEM, port, "sustainer_telem"))
        new_thread.add_combiner(combiner_sustainer)
        if overwrite_rf:
            new_thread.write_frequency(FREQ_SUSTAINER)
        telem_threads_sustainer.append(new_thread)

    # Set up telemetry threads for drone relay
    for port in relay_sources:
        new_thread = mqtt.TelemetryThread(port, log.create_stream(logger.LoggerType.TELEM, port, "relay_telem"))
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
    threads = [broadcast_thread] + all_telemetry_threads
    assert_alive(threads) # Ensure all threads initialized successfully

    # Set up visualization variables
    print_delay = 0.5
    last_print_db = datetime.datetime.now().timestamp()

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

        sustainer_commands, booster_commands = broadcast_thread.get_telem_cmds()

        # handle commands 
        # print(sustainer_commands, booster_commands)

        for command in sustainer_commands:

            telem_threads_sustainer[0].send_command(command)

        for command in booster_commands:
            
            telem_threads_booster[0].send_command(command)
        
        if (is_verbose or not is_visual):
            continue

        sustainer_commands, booster_commands = broadcast_thread.get_telem_cmds()

        # handle commands 
        # print(sustainer_commands, booster_commands)

        for command in sustainer_commands:

            telem_threads_sustainer[0].send_command(command)

        for command in booster_commands:
            
            telem_threads_booster[0].send_command(command)
        
        # Only print occasionally to not flood standard print
        if(datetime.datetime.now().timestamp() - last_print_db > print_delay):
            last_print_db = datetime.datetime.now().timestamp()

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
            broadcast_thread.publish_common(send_data)
