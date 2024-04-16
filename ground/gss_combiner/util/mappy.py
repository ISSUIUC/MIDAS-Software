import paho.mqtt.client as mqtt
import folium
import json
import threading
import time

mymap = folium.Map(location=[0, 0], zoom_start=2)
map_lock = threading.Lock()
client = None

def on_message(client, userdata, message):
    data = json.loads(message.payload.decode())
    print("Received MQTT message:", data)
    
    lat = data['value']['gps_lat']
    lon = data['value']['gps_long']

    print(lat, lon)
    
    if lat is not None and lon is not None:
        with map_lock:
            folium.Marker(location=[lat, lon]).add_to(mymap)


def mqtt_thread():
    global client 
    broker_address = "10.195.167.19"
    topic = "FlightData-Sustainer"
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(broker_address)
    client.subscribe(topic)
    client.loop_forever()


def save_map_thread():
    while True:
        time.sleep(5)
        with map_lock:
            mymap.save("mqtt_map.html")


if __name__ == "__main__":
    mqtt_thread = threading.Thread(target=mqtt_thread)
    mqtt_thread.start()

    save_map_thread = threading.Thread(target=save_map_thread)
    save_map_thread.start()

    try:
        mqtt_thread.join()
        save_map_thread.join()
    except KeyboardInterrupt:
        if client:
            client.disconnect()
        
