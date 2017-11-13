import osmapi

import math
import time
from itertools import groupby
from operator import itemgetter
from random import random
from copy import deepcopy
from datetime import datetime

import pylab as pl
from matplotlib import collections as mc

api = osmapi.OsmApi()

lat, lon = 34.051491, -84.071297

starting_coord = (lon - 0.001, lat + 0.001) #(-84.0736408, 34.0522912)#
end_coord = (lon + 0.001 , lat  - 0.001) #(-84.0843963, 34.0447567)#

notable_coords = []
dist = 10 # km
time_ = 60 # minutes

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


def get_neighbors(node, way_edges):
    ''' Finds the degree of a node along with the node IDs that are adjacent (neighbors) '''

    start_time = datetime.now()
    node_id = node['data']['id']

    print('node id:', node_id)

    # TODO: lksdfka sdjf
    print (len(way_edges))
    edges = list(filter(lambda way_edges: node_id in way_edges, way_edges))
    print(edges)

    edges = [val for sublist in edges for val in sublist]
    print (edges)
    
    edges = list(set(edges))

    edges.sort()

    edges = list(edges for edges, _ in groupby(edges))
    print(edges)
    if len(edges) > 1:
        edges.remove(node_id)

    print(edges)

    print('difference of time for get_neighbors:', datetime.now() - start_time)

    return list(edges)

def a_star_path(start_node, end_node, nodes, way_edges):
    ''' Returns a list of node objs that make up a path between two points''' 
    path = []

    if start_node == end_node:
        return [start_node]

    # This is needed to get rid of all the data collected from the previous times function was run
    for i in nodes:
        try:
            i.pop('prev_node', None)
        except KeyError:
            pass

    queue = []
    start_node['dist'] = 0
    queue.append(start_node)

    end_coord = convert_to_xy((end_node['data']['lon'], end_node['data']['lat']))

    finished = []

    while queue[0]['data']['id'] is not end_node['data']['id']:
        print ('looping in queue of a star')
        # print('start node:', start_node['data']['id'])
        # print('end_node:', end_node['data']['id'])
        # print()
        current_node = queue[0]
        current_coord = convert_to_xy((current_node['data']['lat'], current_node['data']['lon']))
        
        neighbor_ids = get_neighbors(current_node, way_edges)

        neighbors = list(filter(lambda nodes: nodes['data']['id'] in neighbor_ids, nodes))

        # print('BEFORE REMOVAL')
        # for i in neighbors:
        #     print(i['data']['id'])
        
        neighbors = [x for x in neighbors if x['data']['id'] not in finished]
        
        # print('AFTER REMOVAL')
        # for i in neighbors:
        #     print(i['data']['id'])

        # print()
        # print('current node id:', current_node['data']['id'])
        # print('neighbor ids:', neighbor_ids)
        # print()

        start_time = datetime.now()
        for i in neighbors:
            coord = convert_to_xy((i['data']['lon'], i['data']['lat']))
           

            new_dist  = get_dist(coord, current_coord) + current_node['dist']

            try:
                i['prev_node']
            except KeyError:
                i['prev_node'] = current_node

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

        print('diff of time for working with neighbors:', datetime.now() - start_time)
        queue.extend(neighbors)

        finished.append(current_node['data']['id'])

        start_time = datetime.now()
        queue.remove(current_node)
        queue = [x for x in queue if x['data']['id'] not in finished]
        queue = sorted(queue, key = lambda x: x['comb_heur'])
        # print('len of queue:', len(queue))
        queue_ids = [x['data']['id'] for x in queue]

        print('diff of time for working in queue:', datetime.now() - start_time)
        # print('queue:', queue_ids)
        # print()

    path.append(queue[0])
    
    while path[-1] is not start_node:
        print ('looping in path of a_star')
        path.append(path[-1]['prev_node'])
        # print('path i:', path[-1]['data']['id'])

    path = list(reversed(path))

    path_id = [x['data']['id'] for x in path]
    # print('path:', path_id)
    # print()

    return path

def get_leg(starting_node, nodes, way_edges):
    ''' This should return a list of node objects that make up the leg of a path '''

    # The path is made up of several legs
    leg = []

    # starting_node = get_nearest_node(starting_coord, nodes)
    starting_node ['prev_leg_neighbor'] = set()
    leg.append(starting_node)

    leg_finished = False

    just_started = True

    odds_found = False
    first_odd = None
    second_odd = None
    while not(leg_finished):
        print ('looping in get_leg')
        
        odds_found = False
        current_node = leg[-1]
        
        neighbor_ids = get_neighbors(current_node, way_edges)
        neighbors = list(filter(lambda nodes: nodes['data']['id'] in neighbor_ids, nodes))

        neighbors = sorted(neighbors, key=lambda x: x['data']['id']) 
        degree = len(neighbors)
        
        # print('degree', degree)

        if degree % 2 != 0:
            try:
                current_node['odd']
            except KeyError:
                current_node['odd'] = True
            
            if first_odd is None and current_node['odd'] == True:
                first_odd = current_node
            elif current_node['odd'] == True:
                second_odd = current_node

                # print('first odd:', first_odd)
                # print()
                # print('second odd:', second_odd)
                # print()

                odd_path = a_star_path(second_odd, first_odd, nodes, way_edges)
                
                first_odd['odd'] = False
                second_odd['odd'] = False
                
                first_odd = None
                second_odd = None

                # print('len of odd path:', len(odd_path))

                for i in range(len(odd_path)):
                    try:
                        odd_path[i]['prev_leg_neighbor'].add(odd_path[i-1]['data']['id'])
                    except KeyError:
                        odd_path[i]['prev_leg_neighbor'] = set()

                odds_found = True

        if not(odds_found):
            neighbors = [x for x in neighbors if x['data']['id'] not in list(current_node['prev_leg_neighbor'])]
    
            if len(neighbors) == 0:
                # print('no more neighbors')
                leg_finished = True
                break
    
            try:
                neighbors[0]['prev_leg_neighbor'].add(current_node['data']['id'])
            except KeyError:
                neighbors[0]['prev_leg_neighbor'] = set()
                neighbors[0]['prev_leg_neighbor'].add(current_node['data']['id'])
    
            next_node = neighbors[0] 
    
            # print('prev node:', current_node['prev_leg_neighbor'])
            # print('current node:', current_node['data']['id'])
            # print('next node:', next_node['data']['id'])
            leg.append(next_node)
    
            # print('len of leg', len(leg))
            # print()
    
        else:
            leg.extend(odd_path)

        if current_node['data']['id'] is starting_node['data']['id'] and not(just_started):
            # print('current is starting')
            leg_finished = True
            break

        leg_ids = [x['data']['id'] for x in leg]

        # print()
        # print('Leg ids:', leg_ids)
        # print()
        just_started = False

    if leg[0]['data']['id'] != leg[-1]['data']['id']:
        # print('creating return path')
        return_path = a_star_path(leg[-1], leg[0], nodes, way_edges)

        leg.extend(return_path)

    leg = list(map(itemgetter(0), groupby(leg)))

    if len(leg) == 1:
        return []

    return leg

def remove_edges(ways, edges):
    return [item for item in ways if item not in edges]

def get_edges(path):
    '''Takes in a list of node objects and gets the edges between them'''
    edges = []

    for i in range(len(path)):
        try:
            coord = []
            coord.append(path[i]['data']['id'])
            coord.append(path[i+1]['data']['id'])

            edges.append(coord)

        except IndexError:
            pass

    return edges

def get_path_dist(path):
    total = 0

    for i in range(len(path)):
        try:
            coord1 = convert_to_xy((path[i]['data']['lon'], path[i]['data']['lat']))
            coord2 = convert_to_xy((path[i+1]['data']['lon'], path[i+1]['data']['lat']))

            total += get_dist(coord1, coord2)
        except IndexError:
            pass

    return total

def get_path(starting_node, nodes, way_edges, dist_target=0, time_target=0):
    ''' This should returnn a list of node objects that make up the path '''

    path = []

    starting_leg = get_leg(starting_node, nodes, way_edges)

    path.extend(starting_leg)

    if dist_target == 0 and time_target != 0:
        walking_pace = 5 # kmph

        dist_target = walking_pace * (time_target / 60)

    conditions_met = False
    i = 0
    change_made = False

    while not(conditions_met):

        current_node = path[i]
        current_pos = i
        edges = get_edges(path)
        way_edges = remove_edges(way_edges, edges)

        neighbors = get_neighbors(current_node, way_edges)

        new_leg = []
        if len(neighbors) != 0:
            new_leg = get_leg(current_node, nodes, way_edges)

        path[current_pos:current_pos] = new_leg
        
        path = list(map(itemgetter(0), groupby(path)))

        dist = get_path_dist(path)

        print('current index:', i)
        print('len of legs:', len(new_leg))
        print('len of path:', len(path))


        if len(new_leg) != 0:
            change_made = True

        if dist >= dist_target and dist_target != 0:
            print('Dist was > than the dist_target')
            conditions_met = True

        # TODO: Get all the edges from the path already.
        # TODO: Remove edges from ways. (You need to split up the way into two as needed

        i+=1

        if change_made:
            i = 0
            change_made = False
            print('Change has been made')

        if i >= len(path):
            print('Went through entire path and found no more connections to make')
            conditions_met = True

    if path[0]['data']['id'] != path[-1]['data']['id']:
        print('HEY LOOK AT ME')

    return path

def get_edges_from_ways(ways):
    edges = []

    for i in ways:
        nodes = i['data']['nd']
        for j in range(len(nodes)):
            try:
                edges.append([nodes[j], nodes[j+1]])
            except IndexError:
                pass
    print(len(edges))
    #edges.sort()
    #edges = list(edges for edges, _ in groupby(edges))
    print(edges)
    return edges

raw_data = api.Map(min_lon, min_lat, max_lon, max_lat)

nodes = list(filter(lambda raw_data: raw_data['type'] == 'node', raw_data))
ways = list(filter(lambda raw_data: raw_data['type'] == 'way' and 'highway' in raw_data['data']['tag'], raw_data))

# This filters it so that only node that are on roads get counted
nodes_on_way = set()
for i in ways:
    for j in i['data']['nd']:
        nodes_on_way.add(j)

nodes_on_way = list(nodes_on_way)

way_edges = get_edges_from_ways(ways)

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

print('starting node:', starting_node)
print()
print('ending node:', end_node)
print()


leg = get_path(starting_node, nodes, way_edges)#ways)

lines = []
colors = []
points = []
pt_colors = []

print ('THE LEG')

q = 0
w = 1/len(leg)
for i in leg:
    print(i['data']['id'])
    print()
    coord = convert_to_xy((i['data']['lon'],i['data']['lat']))
    q += w
    if q > 1:
        q = 1
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
        colors.append((1,0,0))
    except:
        continue

starting_xy = convert_to_xy(starting_coord)
starting_node_xy = convert_to_xy((starting_node['data']['lon'], starting_node['data']['lat']))

end_xy = convert_to_xy(end_coord)
end_node_xy = convert_to_xy((end_node['data']['lon'], end_node['data']['lat']))

lines.append((starting_xy, starting_node_xy))
colors.append((1,0,0))

points.append(starting_xy)
pt_colors.append((1,1,0))

lines.append((end_xy, end_node_xy))
colors.append((1,0,0))

points.append(end_xy)
pt_colors.append((1,1,0))

draw_paths_mpl(lines, colors, points, pt_colors)
