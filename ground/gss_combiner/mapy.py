import paho.mqtt.client as mqtt
import folium
import json
import time

mymap = folium.Map(location=[0, 0], zoom_start=2)

def on_message(client, userdata, message):
    data = json.loads(message.payload.decode())
    print("Received MQTT message:", data)
    
    lat = data['value']['gps_lat']
    lon = data['value']['gps_long']

    print(lat, lon)
    
    if lat is not None and lon is not None:
        folium.Marker(location=[lat, lon]).add_to(mymap)


# MQTT broker details
broker_address = "localhost"
topic = "MapData"

# Initialize MQTT client
client = mqtt.Client()
client.on_message = on_message
client.connect(broker_address)
client.subscribe(topic)
client.loop_start()

try:
    while True:
        time.sleep(1)
        mymap.save("mqtt_map.html")
except KeyboardInterrupt:
    client.disconnect()
