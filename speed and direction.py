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
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Calculate the reward using a "decreasing quadratic curve"
    DIRECTION_THRESHOLD = 10.0
    k = - 2/ (DIRECTION_THRESHOLD ** 2)
    dir_reward = k * (direction_diff - DIRECTION_THRESHOLD) * (direction_diff + DIRECTION_THRESHOLD)
    
    speed = params.get('speed', 0.0)
    
    # Target speed decays with turning angle (e.g., 3.5 m/s on straights down to 1.0 on tight corners)
    MAX_SPEED = 3.5
    MIN_SPEED = 1.0
    # Map direction_diff in [0, 90] to a target in [MAX_SPEED, MIN_SPEED]
    dd_clipped = min(direction_diff, 90.0)
    target_speed = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * (dd_clipped / 90.0)

    
    # Convert to a smooth factor in (0, 1], e.g., via a Gaussian around target_speed
    sigma = 0.7  # tolerance (tune)
    speed_factor = math.exp(-((speed - target_speed) ** 2) / (2 * sigma ** 2))
    reward = max(dir_reward * (0.5 + 0.5 * speed_factor), 1e-3)
    
    return float(reward)

# maybe can add when track_direction is greater than [SOME VALUE] then reward if the speed is low/ penalise high speed so the car is ready to turn
# the idea is to get the car moving faster at straight roads and slow down for turns
