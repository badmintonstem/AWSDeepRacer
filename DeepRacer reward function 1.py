# Place import statement outside of function (supported libraries: math, random, numpy, scipy, and shapely)
# Example imports of available libraries
#
# import math
# import random
# import numpy
# import scipy
# import shapely

import math

def reward_function(params):
    ###############################################################################
    '''
    Example of using waypoints and heading to make the car point in the right direction
    '''

    # Read input variables
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']

    # Initialize the reward with typical value
    reward = 2.0

    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(((track_direction - heading + 180) % 360) - 180)

    # Calculate the reward using a "decreasing quadratic curve"
    DIRECTION_THRESHOLD = 10.0
    k = - 2.0/ (DIRECTION_THRESHOLD ** 2)
    reward = k * (direction_diff - DIRECTION_THRESHOLD) * (direction_diff + DIRECTION_THRESHOLD)
    
    return float(reward)

# maybe can add when track_direction is greater than [SOME VALUE] then reward if the speed is low/ penalise high speed so the car is ready to turn
# the idea is to get the car moving faster at straight roads and slow down for turns
