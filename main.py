import osmapi

import math
from itertools import groupby
from operator import itemgetter
import argparse

import pylab as pl
from matplotlib import collections as mc

api = osmapi.OsmApi()

ap = argparse.ArgumentParser()
ap.add_argument('-lon', '--longitude', type=float, default = -84.084576,
        help='Longitude coordinates of you startig location')
ap.add_argument('-lat', '--latitude', type=float, default = 34.047401,
        help='Latitude coordinates of you startig location')
ap.add_argument('-d', '--distance', type=float, default = 0,
        help='The target distance of the walk in km. (Default is 0 no limit)')
ap.add_argument('-t', '--time', type=float, default = 0,
        help='The target time of the walk in mins. (Default is 0 no limit)')
ap.add_argument('-w', '--walking_pace', type=float, default = 5,
        help='The walking pace in km per hour. (Default is 5)')

args = vars(ap.parse_args())
lat = args['latitude']
lon = args['longitude']

target_dist = args['distance']
target_time = args['time']
walking_pace = args['walking_pace']

starting_coord = (lon, lat)

# Box width that defines the bounding box for the map
box_width = 0.005

min_lon = lon - (box_width/2)
min_lat = lat - (box_width/2)

max_lon = lon + (box_width/2)
max_lat = lat + (box_width/2)


def convert_to_xy(coord):
    '''
    Converts (longitude, latitude) to km using Equirectangular projection
    returns [x,y]
    '''
    (lon, lat) = coord
    radius = 6371 # km
    theta = (max_lat + min_lat)/2
    x = float(lon) * radius * math.pi / 180 * math.cos(theta * math.pi / 180)
    y = float(lat) * radius * math.pi / 180
    return [x,y]  

def get_dist(coord1, coord2):
    '''
    Gets the distance between two coords
    '''
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def get_nearest_node(coord, nodes):
    '''
    Returns the nearest node object based on longitue latitude coords
    '''
    return min(nodes, key=lambda x: get_dist((coord), (x['data']['lon'], x['data']['lat'])))

def get_neighbors(node, way_edges):
    '''
    Finds the neighbors of a node based on its edges
    returns a list of node ids
    '''
    node_id = node['data']['id']

    edges = list(filter(lambda way_edges: node_id in way_edges, way_edges))
    edges = [val for sublist in edges for val in sublist]
    edges = list(set(edges))
    edges.sort()

    edges = list(edges for edges, _ in groupby(edges))
    if len(edges) > 1:
        edges.remove(node_id)

    return list(edges)

def a_star_path(start_node, end_node, nodes, way_edges):
    '''
    Returns a list of node objects that make up a path between two points using the a* algorithm
    The heuristics used is sum of the distance to the end node and the distance between each node
    returns a list of node objects
    ''' 
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
        current_node = queue[0]
        current_coord = convert_to_xy((current_node['data']['lat'], current_node['data']['lon']))
        
        neighbor_ids = get_neighbors(current_node, way_edges)
        neighbors = list(filter(lambda nodes: nodes['data']['id'] in neighbor_ids, nodes))

        # Gets rid of all neighbors that were already expanded upon (in finished)
        neighbors = [x for x in neighbors if x['data']['id'] not in finished]
        

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

        queue.extend(neighbors)
        finished.append(current_node['data']['id'])

        queue.remove(current_node)

        # Gets rid of any node in queue that are in finished (In theory this should never run)
        queue = [x for x in queue if x['data']['id'] not in finished]
        
        queue = sorted(queue, key = lambda x: x['comb_heur'])

    path.append(queue[0])
    
    while path[-1] is not start_node:
        path.append(path[-1]['prev_node'])

    path = list(reversed(path))

    return path

def get_leg(starting_node, nodes, way_edges):
    '''
    This should return a list of node objects that make up the leg of a path
    A leg here is defined as a list of nodes gained from each cycle of the Euler Circuit Algorithm.
    This also includes the changes for accounting odd nodes with the a* algorithm
    returns list of node objects
    '''

    leg = []

    starting_node ['prev_leg_neighbor'] = set()
    starting_node ['next_leg_neighbor'] = set()
    leg.append(starting_node)

    leg_finished = False

    # Used so while loop doesn't end on first run,
    # as an ending condition is if the starting node is the ending node of leg
    just_started = True

    odds_found = False
    first_odd = None
    second_odd = None

    # This is to reset the nodes from the data of previous function runs
    for i in nodes:
        i['data']['prev_leg_neighbor'] = set() 
        i['data']['next_leg_neighbor'] = set() 

    while not(leg_finished):
        odds_found = False
        current_node = leg[-1]

        neighbor_ids = get_neighbors(current_node, way_edges)
        neighbors = list(filter(lambda nodes: nodes['data']['id'] in neighbor_ids, nodes))

        neighbors = sorted(neighbors, key=lambda x: x['data']['id']) 
        degree = len(neighbors)

        # When the node is fully explored break
        if len(current_node['prev_leg_neighbor']) >= degree:
            break

        if degree % 2 != 0:
            try:
                current_node['odd']
            except KeyError:
                current_node['odd'] = True
            
            if first_odd is None and current_node['odd'] == True:
                first_odd = current_node
            elif current_node['odd'] == True:
                second_odd = current_node

                # Apply a* algorithm if there are 2 nodes of odd degree
                odd_path = a_star_path(second_odd, first_odd, nodes, way_edges)
                
                first_odd['odd'] = False
                second_odd['odd'] = False
                
                first_odd = None
                second_odd = None

                # Sets up neighbors in each node of the a* path
                for i in range(len(odd_path)):
                    try:
                        odd_path[i]['prev_leg_neighbor'].add(odd_path[i-1]['data']['id'])
                    except KeyError:
                        odd_path[i]['prev_leg_neighbor'] = set()
                    except IndexError:
                        pass

                    try:
                        odd_path[i]['next_leg_neighbor'].add(odd_path[i+1]['data']['id'])
                    except KeyError:
                        odd_path[i]['next_leg_neighbor'] = set()
                    except IndexError:
                        pass

                odds_found = True

        if not(odds_found):
            neighbors = [x for x in neighbors if x['data']['id'] not in list(current_node['prev_leg_neighbor'])]
            try:
                # Don't explore edges already travered
                neighbors = [x for x in neighbors if x['data']['id'] not in list(current_node['next_leg_neighbor'])]
            except KeyError:
                pass

            if len(neighbors) == 0:
                leg_finished = True
                break
    
            try:
                neighbors[0]['prev_leg_neighbor'].add(current_node['data']['id'])
            except KeyError:
                neighbors[0]['prev_leg_neighbor'] = set()
                neighbors[0]['prev_leg_neighbor'].add(current_node['data']['id'])
    
            next_node = neighbors[0] 

            try:
                current_node['next_leg_neighbor'].add(next_node['data']['id'])
            except KeyError:
                current_node['next_leg_neighbor'] = set()
                current_node['next_leg_neighbor'].add(next_node['data']['id'])
            
            leg.append(next_node)
    
        else:
            leg.extend(odd_path)
        
        # When looping back to starting node exit, but not if just started
        if current_node['data']['id'] is starting_node['data']['id'] and not(just_started):
            leg_finished = True
            break

        just_started = False

    # if start and end nodes are not the same create path between the two.
    if leg[0]['data']['id'] != leg[-1]['data']['id']:
        return_path = a_star_path(leg[-1], leg[0], nodes, way_edges)
        leg.extend(return_path)

    leg = list(map(itemgetter(0), groupby(leg)))

    if len(leg) == 1:
        return []

    return leg

def remove_edges(ways, edges):
    '''
    Removes the edges from ways (list of edges)
    returns list of edges
    '''
    return [item for item in ways if item not in edges]

def get_edges(path):
    '''
    Takes in a list of node objects and gets the edges between them
    returns a list of lines (two points)
    '''
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
    '''
    Returns the distance (km) of a path (list of node objects)
    '''
    total = 0

    for i in range(len(path)):
        try:
            coord1 = convert_to_xy((path[i]['data']['lon'], path[i]['data']['lat']))
            coord2 = convert_to_xy((path[i+1]['data']['lon'], path[i+1]['data']['lat']))

            total += get_dist(coord1, coord2)
        except IndexError:
            pass

    return total

def get_path(starting_node, nodes, way_edges, dist_target=0, time_target=0, walking_pace=5):
    '''
    Finds the list of node objects that make up the path
    returns list of node objects, dist (km), time (mins)
    '''

    # Choose the smaller metric between time and distance
    if dist_target == 0 and time_target != 0:
        dist_target = walking_pace * (time_target / 60)
    elif dist_target != 0 and time_target != 0:
        dist_target2 = walking_pace * (time_target / 60)
        dist_target = dist_target if dist_target < dist_target2 else dist_target2

    path = []

    starting_leg = get_leg(starting_node, nodes, way_edges)
    path.extend(starting_leg)


    conditions_met = False
    change_made = False

    dist = 0

    i = 0
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

        i+=1

        if len(new_leg) != 0:
            change_made = True

        # Stop when the distance target is met or exceeded
        if dist >= dist_target and dist_target != 0:
            conditions_met = True

        if change_made:
            #i = 0
            change_made = False
            print('len of path:', len(path))

        if i >= len(path):
            conditions_met = True

    time = (dist/walking_pace) * 60
    return path, dist, time

def get_edges_from_ways(ways):
    '''
    Converts the ways object from osm to edges
    returns list of lines (node id, node id)
    '''
    edges = []

    for i in ways:
        nodes = i['data']['nd']
        for j in range(len(nodes)):
            try:
                edges.append([nodes[j], nodes[j+1]])
            except IndexError:
                pass
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
nodes = list(filter (lambda nodes: nodes['data']['id'] in nodes_on_way, nodes))

way_edges = get_edges_from_ways(ways)

starting_node = get_nearest_node(starting_coord, nodes)

path, path_dist, path_time = get_path(starting_node, nodes, way_edges, target_dist, target_time, walking_pace)

################################DISPLAYING##################################
def get_map_lines(ways):
    '''
    Converts ways node objects into a list of lines (coords) and colors for drawing
    returns list of lines (coord, coord), list of colors ((r, g, b))
    '''
    lines = []
    colors = []
    for i in ways:
        node_list = i['data']['nd']
        
        # Colors range from 0 - 1 corresponding to 0 to 255 for rgb and alpha
        # Roads are colored blue with 0.5 transparency
        c = (0, 0, 1, 0.5)
        
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
    return lines, colors

def draw_paths_mpl(lines, line_colors, points=[], point_colors=[]):
    '''
    Displays in a window the specified series of lines and colors.
    Also displays points as hexagons and their respectively specified colors if specified
    '''
    fig, ax = pl.subplots()

    pts = mc.RegularPolyCollection(
            numsides=6,
            rotation=0,
            sizes=(50,),
            facecolors = point_colors,
            edgecolors = (0,0,0,1), 
            offsets=points,
            transOffset = ax.transData,
            )
    
    lc = mc.LineCollection(lines, colors=line_colors, linewidths=2)

    ax.add_collection(lc)
    ax.add_collection(pts)
    ax.autoscale()
    ax.margins(0.1)

    pl.show()

print('starting location (lon, lat)')
print('(',starting_node['data']['lon'], ',' ,starting_node['data']['lat'],')')
print()

print ('Path coords:')
for i in path:
    print(i['data']['lon'],',',i['data']['lat'])

print()
print('Dist:', path_dist, 'km')
print('Time:', path_time, 'mins')

# Variables for displaying the map
lines = []
colors = []
points = []
pt_colors = []

# Adds markers on a map which gradually changes from green to red
c = 0
try:
    color_change = 1/len(path)
except ZeroDivisionError:
    color_change = 0
for i in path:
    coord = convert_to_xy((i['data']['lon'],i['data']['lat']))
    c += color_change
    if c > 1:
        c = 1
    points.append(coord)
    pt_colors.append((c,1-c,0, 0.5))

map_lines, map_colors = get_map_lines(ways)

lines.extend(map_lines)
colors.extend(map_colors)

# Adds a red line that makes up the path
# for i in range(len(points)):
#     try:
#         lines.append((points[i], points[i+1]))
#         colors.append((1,0,0))
#     except:
#         continue

# Since the starting node and starting point may be different the code below marks the actual location
# as well as marking the nearest node, and draws a red line between the two
starting_xy = convert_to_xy(starting_coord)
starting_node_xy = convert_to_xy((starting_node['data']['lon'], starting_node['data']['lat']))

lines.append((starting_xy, starting_node_xy))
colors.append((1,0,0))

points.append(starting_xy)
pt_colors.append((1,1,0))

draw_paths_mpl(lines, colors, points, pt_colors)


