from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
    print(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
import carla
import sys
sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client("localhost", 2000)
client.set_timeout(10)
world = client.load_world('Town04')
amap = world.get_map()
sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()
file_handle_spawn_points =open('spawn_points_Town04.txt',mode='w+')
# print(len(spawn_points))
ii = 0
for s in spawn_points:
    file_handle_spawn_points.write("spawn_point " + str(ii) + ":" + str(spawn_points[ii]) + "\n")
    ii = ii + 1
# print(str(spawn_points[0]))
# file_handle_spawn_points.write(spawn_points)

origin = carla.Location(spawn_points[0].location)
destination = carla.Location(spawn_points[80].location)
print("\n=====================================spawn_points=================================")
print(spawn_points[0])


# there are other funcations can be used to generate a route in GlobalRoutePlanner.
w1 = grp.trace_route(origin, destination) 
i = 0
file_handle=open('reference_line_Town04.txt',mode='w+')
for w in w1:
    if i % 10 == 0:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
        color=carla.Color(r=255, g=0, b=0), life_time=120.0,
        persistent_lines=True)
    else:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
        color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
        persistent_lines=True)
    i += 1
    location_str = str(w[0].transform.location.x) + ", " + str(w[0].transform.location.y)+ "\n";
    # print(location_str)
    file_handle.write(location_str)
file_handle.close()
# print("===========================================")
# print(i)
# x = w[0].transform.location;
# print(type(x))
# print(dir(x))
# print(w[0].transform.location.x, w[0].transform.location.y, w[0].transform.location.z)
# print(a)
# # print(type(spawn_points[0]))
# # print(dir(spawn_points[0]))
# print(spawn_points[50])
# print(spawn_points[50].location.x)
# print(a)
# print(w[0])
# print(origin)
