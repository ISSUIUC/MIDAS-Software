# import folium

# def main():
#     map_osm = folium.Map(location=[51.5074, -0.1278], zoom_start=10)
#     folium.TileLayer('openstreetmap').add_to(map_osm)
#     map_osm.save('osm_map.html')
# if __name__ == "__main__":
#     main()


import folium

# Latitude and Longitude of Champaign County
champaign_county_coords = (40.1397, -88.2001)

# Create map centered around Champaign County
mymap = folium.Map(location=champaign_county_coords, zoom_start=10)

# List of points to plot (latitude, longitude)
points = [
    (40.1164, -88.2434),  # Point 1
    (40.1234, -88.2615),  # Point 2
    (40.1629, -88.1506),  # Point 3
    (40.2284, -88.2614),  # Point 4
    (40.0971, -88.2017)   # Point 5
]

# Add markers for each point
for point in points:
    folium.Marker(location=point).add_to(mymap)

# Save the map as an HTML file
mymap.save("champaign_county_map.html")
