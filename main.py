import osmapi

import math
from random import random

import pylab as pl
from matplotlib import collections as mc

api = osmapi.OsmApi()

lat, lon = 34.051491, -84.071297

starting_coord = (lon - 0.001, lat + 0.001)

notable_coords = []
dist = 10 # km
time = 60 # minutes

box_width = 0.005
box_height = 0.005

min_lon = lon - (box_width/2)
min_lat = lat - (box_height/2)

max_lon = lon + (box_width/2)
max_lat = lat + (box_height/2)

def draw_paths_mpl(lines, line_colors, points=[], point_colors=[]):
    fig, ax = pl.subplots()
    lc = mc.LineCollection(lines, colors=line_colors, linewidths=2)

    pts = mc.RegularPolyCollection(
            numsides=6,
            rotation=0,
            sizes=(50,),
            facecolors = point_colors,
            edgecolors = (0,0,0,1), 
            offsets=points,
            transOffset = ax.transData,
            )
    
    ax.add_collection(lc)
    ax.add_collection(pts)
    ax.autoscale()
    ax.margins(0.1)

    pl.show()

def convert_to_xy(coord):
    (lon, lat) = coord
    radius = 6371 # km
    theta = (max_lat + min_lat)/2
    x = float(lon) * radius * math.pi / 180 * math.cos(theta * math.pi / 180)
    y = float(lat) * radius * math.pi / 180
    return (x,y)

def get_dist(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def get_nearest_node(starting_coord, nodes):
    '''Returns the nearest node object based on longitue latitude coords '''
    return min(nodes, key=lambda x: get_dist((starting_coord), (x['data']['lon'], x['data']['lat'])))

def get_neighbors(node, ways):
    ''' Finds the degree of a node along with the node IDs that are adjacent (neighbors) '''

    node_id = node['data']['id']

    # This should contain all the ways the node is on
    nd_ways = list(filter(lambda ways: node_id in ways['data']['nd'], ways))

    neighbor_nodes = set()

    for i in nd_ways: 
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
    ''' This should return a list of node objects that make up the path '''

    # This is just a list of nodes representing the path.
    # The path should contain duplicate nodes when that node is visited twice
    path = []

    starting_node = get_nearest_node(starting_coord, nodes)
    path.append(starting_node)


    current_index = path[0]
    previous_node = None

    changes_made = True
    while changes_made:
        
        current_node = path[current_index]
        
        neighbor_ids = get_neighbors(current_node, ways)

        degree = len(neighbor_ids)

        if degree % 2 != 0:
            # TODO: Consider the odd degree
            pass

        previous_node = current_node
        current_index += 1

    return path

raw_data = api.Map(min_lon, min_lat, max_lon, max_lat)

nodes = list(filter(lambda raw_data: raw_data['type'] == 'node', raw_data))
ways = list(filter(lambda raw_data: raw_data['type'] == 'way' and 'highway' in raw_data['data']['tag'], raw_data))
relations = list(filter(lambda raw_data: raw_data['type'] == 'relation', raw_data))

print('total data amount:', len(raw_data))
print('Number of nodes:', len(nodes))
print('Number of ways:', len(ways))
print('Number of relations:', len(relations))

walkable_tags = [
        'secondary',
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
        'path'
        ]

walkable_ways = list(filter(lambda ways: ways['data']['tag']['highway'] in walkable_tags, ways))

print('Number of walkable ways:', len(walkable_ways))



###################### Everything From this point on is for the purposes of drawing #####################
lines = []
colors = []

points = []
pt_colors = []

for i in ways:
    node_list = i['data']['nd']
    
    c = (random() * 0.25, random(), random())
    
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

starting_node = get_nearest_node(starting_coord, nodes)

starting_xy = convert_to_xy(starting_coord)
starting_node_xy = convert_to_xy((starting_node['data']['lon'], starting_node['data']['lat']))

lines.append((starting_xy, starting_node_xy))
colors.append((1,0,0))

points.append(starting_xy)
pt_colors.append((1,1,0))

draw_paths_mpl(lines, colors, points, pt_colors)
