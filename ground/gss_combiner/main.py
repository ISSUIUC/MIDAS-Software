import sys

import mqtt
import combiner
import threading

def assert_alive(threads: list[threading.Thread]):
    for thread in threads:
        assert thread.is_alive()

if __name__ == "__main__":
    threads = []
    if len(sys.argv) == 5:
        
        if sys.argv[1] == "--sustainer" and sys.argv[3] == "--booster":
            print("Arguments OK")

            if sys.argv[2] == "E":
                sustainer_sources = []
            else:
                sustainer_sources = sys.argv[2].split(",")

            if sys.argv[4] == "E":
                booster_sources = []
            else:
                booster_sources = sys.argv[4].split(",")



            print("Using sustainer sources: ", sustainer_sources)
            print("Using booster sources: ", booster_sources)

            telem_threads_booster = []
            telem_threads_sustainer = []

            for port in booster_sources:
                new_thread = mqtt.TelemetryThread(port, "localhost", "FlightData-All")
                telem_threads_booster.append(new_thread)

            for port in sustainer_sources:
                new_thread = mqtt.TelemetryThread(port, "localhost", "FlightData-All")
                telem_threads_sustainer.append(new_thread)


            for thd in telem_threads_booster:
                thd.start()

            for thd in telem_threads_sustainer:
                thd.start()

            combiner_sus = combiner.TelemetryCombiner("Sustainer", telem_threads_sustainer)
            combiner_boo = combiner.TelemetryCombiner("Booster", telem_threads_booster)


            broadcast_thread = mqtt.MQTTThread([combiner_sus, combiner_boo], "localhost")
            broadcast_thread.start()

            threads = [broadcast_thread] + telem_threads_booster + telem_threads_sustainer

            assert_alive(threads)
            
            print("\nTelemetry system initialized successfully!\nListening for packets..")

            while True:
                assert_alive(threads)
                pass # :)

        else:
            print("Incorrect format. EXAMPLE:\n./main.py --sustainer COM1,COM2 --booster COM3,COM4")
    else:
        print("Incorrect format. EXAMPLE:\n./main.py --sustainer COM1,COM2 --booster COM3,COM4")



# com0com setup:
# COM1 <-> COM16
# COM2 <-> COM17
# COM18 <-> COM19
# COM20 <-> COM21
# & C:/Python311/python.exe c:/Users/mpkar/Documents/ISS/MIDAS-Software/ground/gss_combiner/main.py --sustainer COM1,COM2 --booster COM18,COM20
