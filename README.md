# Walking Trail Generation
Program should automatically generate walking trails using OpenStreetMap data

The project was done as extra credit project for the 3012 Applied Combinatorics Class at Georgia Tech

Run main.py with the arguments below

'''
usage: main.py [-h] [-lon LONGITUDE] [-lat LATITUDE] [-d DISTANCE] [-t TIME]
               [-w WALKING_PACE]

optional arguments:
  -h, --help            show this help message and exit
  -lon LONGITUDE, --longitude LONGITUDE
                        Longitude coordinates of you startig location
  -lat LATITUDE, --latitude LATITUDE
                        Latitude coordinates of you startig location
  -d DISTANCE, --distance DISTANCE
                        The target distance of the walk in km. (Default is 0
                        no limit)
  -t TIME, --time TIME  The target time of the walk in mins. (Default is 0 no
                        limit)
  -w WALKING_PACE, --walking_pace WALKING_PACE
                        The walking pace in km per hour. (Default is 5)
'''

The output will start as a series of prints stating the path length (in terms of how many vertices or nodes it has)

Once the path is finished, the output will print out a series of longitude latitude coordinates that make up the path and the distance and time estimates for the path

A display should then pop up showing the road map in transparent blue, and the path with hexagonal markers starting from green and ending in red.
