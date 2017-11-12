import osmapi

import math
from random import random

import pylab as pl
from matplotlib import collections as mc

api = osmapi.OsmApi()

lat, lon = 34.051491, -84.071297

starting_coord = (lon - 0.001, lat + 0.001)
end_coord = (lon + 0.001 , lat  - 0.001)

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
    return [x,y]  

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

def a_star_path(start_node, end_node, nodes, ways):
    ''' Returns a list of node ids that make up a path between two points''' 
    path = []

    if start_node == end_node:
        return [start_node]

    queue = []
    start_node['dist'] = 0
    queue.append(start_node)

    end_coord = convert_to_xy((end_node['data']['lon'], end_node['data']['lat']))

    finished = []

    while queue[0] is not end_node:
        current_node = queue[0]
        current_coord = convert_to_xy((current_node['data']['lat'], current_node['data']['lon']))
        
        neighbor_ids = get_neighbors(current_node, ways)

        neighbors = list(filter(lambda nodes: nodes['data']['id'] in neighbor_ids, nodes))
        for i in neighbors:
            coord = convert_to_xy((i['data']['lon'], i['data']['lat']))
           

            new_dist  = get_dist(coord, current_coord) + current_node['dist']
            try:
                if new_dist < i['dist']:
                    i['dist'] = new_dist
                    i['prev_node'] = current_node
            except KeyError:
                i['dist'] = new_dist
                i['prev_node'] = current_node
            
            
            try:
                i['end_dist']
            except KeyError:
                i['end_dist'] = get_dist(coord, end_coord)

            i['comb_heur'] = i['end_dist'] + i['dist']


        neighbors = sorted(neighbors, key=lambda x: x['comb_heur']) 

        queue.extend(neighbors)

        finished.append(current_node)

        queue.remove(current_node)
        queue = [x for x in queue if x not in finished]
        queue = sorted(queue, key = lambda x: x['comb_heur'])
        print('hey', len(queue))

    path.append(end_node)
    
    while path[-1] is not start_node:
        path.append(path[-1]['prev_node'])

    path = list(reversed(path))

    return path

def get_leg(starting_node, nodes, ways):
    ''' This should return a list of node objects that make up the leg of a path '''

    # The path is made up of several legs
    leg = []

    # starting_node = get_nearest_node(starting_coord, nodes)
    leg.append(starting_node)

    leg_finished = False

    prev_node = None
    while not(leg_finished):
        
        current_node = leg[-1]
        
        neighbor_ids = get_neighbors(current_node, ways)

        neighbors = list(filter(lambda nodes: nodes['data']['id'] in neighbor_ids, nodes))
      
        try:
            neighbors = [x for x in neighbors if x not in current_node['explored_neighbors']]
        except KeyError:
            pass

        if prev_node is not None and len(neighbors) > 1:
            try:
                neighbors.remove(prev_node)
            except ValueError:
                pass

        # # This should get rid of any neighbors that already show up on the leg of the journey (not the overall path)
        # if leg[0] in neighbors and :
        #     neighbors = [x for x in neighbors if x not in leg]
        #     neighbors.append(leg[0]) # need to add back starting node that got removed from neighbors
        # else:
        #     neighbors = [x for x in neighbors if x not in leg]

        neighbors = sorted(neighbors, key=lambda x: x['data']['id']) 
        
        degree = len(neighbors)

        if degree == 0:
            leg_finished = True
            break
        
        next_node = neighbors[0]
        prev_node = current_node

        if degree % 2 != 0:
            try:
                if next_node['odd'] is False:
                    leg.append(next_node)
                    continue
            except KeyError:
                next_node['odd'] = True

            odd_start = None
            odd_end = None
    
            for i in reversed(leg):
                try:
                    if i['odd'] is True:
                        odd_end = i
    
                    odd_start = next_node
    
                except KeyError:
                    pass

            if odd_start is not None and odd_end is not None:
                odd_path = a_star_path (odd_start, odd_end, nodes, ways)
                odd_start['odd'] = False
                odd_end['odd'] = False
                
                leg.extend(odd_path)
                continue
        
        try:
            current_node['explored_neighbors'].append(next_node)
        except KeyError:
            current_node['explored_neighbors'] = []
            current_node['explored_neighbors'].append(next_node)

        print(len(leg))
        leg.append(next_node)
        
    return leg

def get_path(starting_node, nodes, ways):
    ''' This should returnn a list of node objects that make up the path '''

    path = []

    starting_leg = get_leg(starting_node, nodes, ways)

    while True:
       pass 


    return path

raw_data = api.Map(min_lon, min_lat, max_lon, max_lat)

nodes = list(filter(lambda raw_data: raw_data['type'] == 'node', raw_data))
ways = list(filter(lambda raw_data: raw_data['type'] == 'way' and 'highway' in raw_data['data']['tag'], raw_data))

# This filters it so that only node that are on roads get counted
nodes_on_way = set()
for i in ways:
    for j in i['data']['nd']:
        nodes_on_way.add(j)

nodes_on_way = list(nodes_on_way)

nodes = list(filter (lambda nodes: nodes['data']['id'] in nodes_on_way, nodes))


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

starting_node = get_nearest_node(starting_coord, nodes)
end_node = get_nearest_node(end_coord, nodes)

leg = get_leg(starting_node, nodes, ways)

lines = []
colors = []
points = []
pt_colors = []

print ('THE LEG')

q = 0
w = 1/len(leg)
for i in leg:
    print(i)
    print()
    coord = convert_to_xy((i['data']['lon'],i['data']['lat']))
    q += w
    if q > 1:
        q = 1
    coord[0] += q/5
    points.append(coord)
    pt_colors.append((0,0,q, q))

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

for i in range(len(points)):
    try:
        lines.append((points[i], points[i+1]))
        colors.append((0,1,1))
    except:
        continue

starting_xy = convert_to_xy(starting_coord)
starting_node_xy = convert_to_xy((starting_node['data']['lon'], starting_node['data']['lat']))

lines.append((starting_xy, starting_node_xy))
colors.append((1,0,0))

points.append(starting_xy)
pt_colors.append((1,1,0))


draw_paths_mpl(lines, colors, points, pt_colors)
