import sys

import mqtt
import combiner

if __name__ == "__main__":
    if len(sys.argv) == 5:
        
        if sys.argv[1] == "--sustainer" and sys.argv[3] == "--booster":
            sustainer_sources = sys.argv[2].split(",")
            booster_sources = sys.argv[4].split(",")

            t1 = mqtt.TelemetryThread(sustainer_sources[0], "mqtt://localhost:1883", "FlightData-All")
            t2 = mqtt.TelemetryThread(sustainer_sources[1], "mqtt://localhost:1883", "FlightData-All")
            t3 = mqtt.TelemetryThread(booster_sources[0], "mqtt://localhost:1883", "FlightData-All")
            t4 = mqtt.TelemetryThread(booster_sources[1], "mqtt://localhost:1883", "FlightData-All")

            combiner_sus = combiner.TelemetryCombiner("FlightData-Sustainer", [t1,t2])
            combiner_boo = combiner.TelemetryCombiner("FlightData-Booster", [t3,t4])

            broadcast_thread = mqtt.MQTTThread([combiner_sus, combiner_boo], "mqtt://localhost:1883")

            t1.run()
            t2.run()
            t3.run()
            t4.run()
            combiner_sus.run()
            combiner_boo.run()
            broadcast_thread.run()



        else:
            print("Incorrect format. EXAMPLE:\n./main.py --sustainer COM1,COM2 --booster COM3,COM4")
    else:
        print("Incorrect format. EXAMPLE:\n./main.py --sustainer COM1,COM2 --booster COM3,COM4")

