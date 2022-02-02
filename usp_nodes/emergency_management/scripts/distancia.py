#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
# Author: Carlos Capitán Fernández
#import numpy as np  

#p1 = np.array([0,0,0])
#p2 = np.array([0,0,1])
#distancia = np.linalg.norm(p1-p2)
#print(distancia)

import math

def calculate_distance(starting_x, starting_y, destination_x, destination_y):
    distance = math.hypot(destination_x - starting_x, destination_y - starting_y)  # calculates Euclidean distance (straight-line) distance between two points
    return distance

def calculate_path(selected_map):
    total_distance = 0
    current_point = selected_map[0]
    for next_point in selected_map[1:]:
        current_distance = calculate_distance(
            current_point[0], current_point[1],
            next_point[0], next_point[1]
        )
        print(current_point, 'to', next_point, '=', current_distance)
        total_distance += current_distance
        current_point = next_point
    return total_distance

selected_map = [(12, 34), (45, -55), (-89, 33), (60, 12)]
distance = calculate_path(selected_map)

print ('Total Distance =', distance)