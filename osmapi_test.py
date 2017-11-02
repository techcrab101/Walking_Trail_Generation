import osmapi

api = osmapi.OsmApi()

lat, lon = 34.051491, -84.071297

box_width = 0.01
box_height = 0.01

min_lon = lon - (box_width/2)
min_lat = lat - (box_height/2)

max_lon = lon + (box_width/2)
max_lat = lat + (box_height/2)

x = api.Map(min_lon, min_lat, max_lon, max_lat)
print(x)
