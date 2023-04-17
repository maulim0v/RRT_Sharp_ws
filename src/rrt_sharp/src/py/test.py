import sys
from pathlib import Path
from enum import Enum
import json
import numpy as np
import matplotlib.pyplot as plt

# Map types to encodings (NO_DATA->0, VALID->1...)
Encoding = Enum('Encoding', ['NO_DATA', 'VALID', 'NOISE', 'NOISE_INFERRED', 'INFERRED'])

PATH_TO_SHARED_LIB = str(Path(__file__).resolve().parent.parent.parent) + '/build/'
sys.path.append(PATH_TO_SHARED_LIB)

print(PATH_TO_SHARED_LIB)

from rrt_sharp_py import *

print('Successful import of rrt_sharp_py module')

def load_layers(file):
    # Sample read 1 map
    f = open(file)
    data = json.load(f)

    # Map grid/cell xy resolution
    map_resolution = data['map']['info']['resolution']

    # Number of grid cells in x and y
    map_num_x = data['map']['info']['numX']
    map_num_y = data['map']['info']['numY']

    # Left/buttom corner
    map_origin_x = data['map']['info']['poseX']
    map_origin_y = data['map']['info']['poseY']

    # reference to map core data
    map_object = data['map']['object']
    map_observed = data['map']['observed']
    map_terrain = data['map']['terrain']

    object_np = np.zeros((map_num_x, map_num_y))
    observed_np = np.zeros((map_num_x, map_num_y))
    terrain_np = np.zeros((map_num_x, map_num_y))
    for i in range(len(map_object)):
       indX = int(i / map_num_y)
       indY = int(i % map_num_y)

       # Populate object height map
       object_np[indY, indX] = map_object[i]

       observed_np[indY, indX] = map_observed[i]

       if map_observed[i] == Encoding.NOISE:
          observed_np[indY, indX] = Encoding.NO_DATA
       elif map_observed[i] == Encoding.NOISE_INFERRED:
          observed_np[indY, indX] = Encoding.INFERRED

       # Populate terrain map
       terrain_np[indY, indX] = map_terrain[i]

    f.close()

    return object_np, observed_np, terrain_np, map_origin_x, map_origin_y, map_resolution, map_num_x, map_num_y

object_map, observed_map, terrain_map, origin_x, origin_y, resolution, num_x, num_y = load_layers('/home/maulimov/ocrl_data/file150.json')

#plt.imshow(object_map, origin ='lower')
#plt.show()

#plt.imshow(observed_map, origin ='lower')
#plt.show()

# Create Map dimension
rrt_sharp = RRTSharp([2, 3, 4])
rrt_sharp.set_map_info(origin_x, origin_y, resolution, num_x, num_y)
rrt_sharp.set_object_map(object_map.flatten().tolist())
rrt_sharp.set_observed_map(observed_map.astype(int).flatten().tolist())
rrt_sharp.set_terrain_map(terrain_map.flatten().tolist())
rrt_sharp.set_start_state(227.0, -143.0, 180.0)
rrt_sharp.set_goal_state(114.0, -250.0, 300.0)
rrt_sharp.set_stop_radius(5.0)
rrt_sharp.init()
rrt_sharp.run()

traj_x = list(rrt_sharp.get_trajectory_x())
traj_y = list(rrt_sharp.get_trajectory_y())

search_space_x = list(rrt_sharp.get_search_space_x())
search_space_y = list(rrt_sharp.get_search_space_y())

plt.scatter(search_space_x, search_space_y)

plt.plot(traj_x, traj_y)
plt.show()

