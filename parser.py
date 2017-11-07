import osmapi

import math
from random import random

import pylab as pl
from matplotlib import collections as mc

api = osmapi.OsmApi()

lat, lon = 34.051491, -84.071297

starting_coord = (lon, lat)

box_width = 0.005
box_height = 0.005

min_lon = lon - (box_width/2)
min_lat = lat - (box_height/2)

max_lon = lon + (box_width/2)
max_lat = lat + (box_height/2)

def convert_to_xy(coord):
    (lon, lat) = coord
    radius = 6371 # km
    theta = (max_lat + min_lat)/2
    x = float(lon) * radius * math.pi / 180 * math.cos(theta * math.pi / 180)
    y = float(lat) * radius * math.pi / 180
    return (x,y)

def get_dist(coord1, coord2):
    dist = 0
    return dist

def get_nearest_node(starting_coord, nodes):
    '''Returns the nearest node based on longitue latitude coords '''
    return min(nodes, key=lambda x: )

    pass

def get_neighbors(node, ways):
    ''' Finds the degree of a node along with the nodes that are adjacent (neighbors) '''

    node_id = node['data']['id']

    # This should contain all the ways the node is on
    nd_ways = list(filter(lambda ways: node_id in ways['data']['nd'], ways))

    neighbor_nodes = set()

    for i in nd_ways 
        pos = i['data']['nd'].index(node_id)

        try:
            neighbor_nodes.add(i['data']['nd'][pos + 1])
        except IndexError:
            pass
        try:
            neighbor_nodes.add(i['data']['nd'][pos - 1])
        except IndexError:
            pass

    return list(neighbor_nodes)

def get_path(starting_coord, nodes, ways):
    ''' This should return a list of nodes that make up the path '''
    
    pass

raw_data = api.Map(min_lon, min_lat, max_lon, max_lat)


print('total data amount:', len(raw_data))

nodes = list(filter(lambda raw_data: raw_data['type'] == 'node', raw_data))
ways = list(filter(lambda raw_data: raw_data['type'] == 'way' and 'highway' in raw_data['data']['tag'], raw_data))
relations = list(filter(lambda raw_data: raw_data['type'] == 'relation', raw_data))

print('Number of nodes:', len(nodes))
print('Number of ways:', len(ways))
print('Number of relations:', len(relations))

walkable_tags = ['secondary',
        'tertiary',
        'unclassified',
        'residential',
        'secondary_link',
        'tertiary_link',
        'living_street',
        'pedestrian',
        'track',
        'road',
        'footway',
        'bridleway',
        'steps',
        'path']

def draw_paths_mpl(lines, colors):
    lc = mc.LineCollection(lines, colors=colors, linewidths=2)

    fig, ax = pl.subplots()
    ax.add_collection(lc)
    ax.autoscale()
    ax.margins(0.1)

    pl.show()

walkable_ways = list(filter(lambda ways: ways['data']['tag']['highway'] in walkable_tags, ways))

lines = []
colors = []

for i in walkable_ways:
    print(i)
    print()
    node_list = i['data']['nd']
    
    print(node_list)
    print()
   
    c = (random(), random(), random())
    
    for j in range(len(node_list)):
        try:
            node_list[j+1]
        except:
            break
        prev_node = list(filter(lambda nodes: nodes['data']['id'] == node_list[j], nodes))[0]
        next_node = list(filter(lambda nodes: nodes['data']['id'] == node_list[j+1], nodes))[0]

        prev_coord = (prev_node['data']['lon'], prev_node['data']['lat'])
        next_coord = (next_node['data']['lon'], next_node['data']['lat'])

        prev_coord = convert_to_xy(prev_coord)
        next_coord = convert_to_xy(next_coord)

        lines.append((prev_coord, next_coord))
        colors.append(c)
        
draw_paths_mpl(lines, colors)
