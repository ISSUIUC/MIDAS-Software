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
            sustainer_sources = sys.argv[2].split(",")
            booster_sources = sys.argv[4].split(",")

            print("Using sustainer sources: ", sustainer_sources)
            print("Using booster sources: ", booster_sources)

            telem_threads_booster = [mqtt.TelemetryThread(src, "localhost", "FlightData-All") for src in booster_sources]
            telem_threads_sustainer = [mqtt.TelemetryThread(src, "localhost", "FlightData-All") for src in booster_sources]

            t1 = mqtt.TelemetryThread(sustainer_sources[0], "localhost", "FlightData-All")
            t2 = mqtt.TelemetryThread(sustainer_sources[1], "localhost", "FlightData-All")
            t3 = mqtt.TelemetryThread(booster_sources[0], "localhost", "FlightData-All")
            t4 = mqtt.TelemetryThread(booster_sources[1], "localhost", "FlightData-All")

            t1.start()
            t2.start()
            t3.start()
            t4.start()

            combiner_sus = combiner.TelemetryCombiner("Sustainer", [t1,t2])
            combiner_boo = combiner.TelemetryCombiner("Booster", [t3,t4])


            broadcast_thread = mqtt.MQTTThread([combiner_sus, combiner_boo], "localhost")
            broadcast_thread.start()

            threads = [t1,t2,t3,t4,broadcast_thread]

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
