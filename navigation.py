# Packages (libraries) required
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Custom modules
import initial_map as mapping

# Unique radio link identifier between computer and Craziflie UAS
URI = 'radio://0/80/2M/E7E7E7E7E9'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Control constants
RHT = 0 # Going
LHT = 1 # Returning
TRAFFIC_TYPE = LHT
# Define speed increment
VELOCITY = 0.2 # m/s
TOTAL_WIDTH = 2
DISTANCE_TO_WALL = TOTAL_WIDTH / 4
OTHER_TRAFFIC_DISTANCE = 0
# STATES = {}

# Function that checks if the 'range' value is smaller than 0.2. 
# It return a Boolean True is so, and False if not
def is_close_to_obstacle(range):
    MAX_LIMIT = 0.9  # m
    MIN_LIMIT = 0.7 # m
    if range is None:
        return False
    else:
        return range > MIN_LIMIT and range < MAX_LIMIT

def inside_boundaries(occupancy_grid, y=0, x=0):
    grid_size = occupancy_grid.shape
    return y in range(grid_size[0]) and x in range(grid_size[1])

def perform_mision():
    
    global DISTANCE_TO_WALL
    global OTHER_TRAFFIC_DISTANCE
    global TRAFFIC_TYPE
    global RHT, LHT

    map_occupancy_grid = mapping.build_initial_map()
    last_y, last_x = 0, 0
    # current_y, current_x = 0, 20

    # Create a synchronous connection with the UAS, UAS is now known as 'scf'
    with SyncCrazyflie(URI) as scf:
        
        # Create a motion commandor (tools that let's you change the motion of the UAS) for 'scf'
        with MotionCommander(scf, 0.2) as motion_commander:
            # -------------------------------------
            # Motion #1: Take off
            # -------------------------------------
            print('Taking off! -by default up to 0.3m-')
            
            # print('Moving up 1.0m at 0.2m/s')
            # motion_commander.up(1.0, 0.2)
            
            right_traffic = TRAFFIC_TYPE == 0
            print('TRAFFIC TYPE', TRAFFIC_TYPE)
            # Wait a bit (i.e. 5 seconds) before doing the next step
            time.sleep(5)

            # Define a multirange object 'multi-ranger'
            with Multiranger(scf) as multi_ranger:
                
                # Define a Boolean that goes to False to stop the mission
                keep_flying = True

                # Continue performing the loop while 'keep_flying' Booelan is True
                forward_velocity = 0.5
                
                current_y, current_x = 0, 20 # Current position in the map (y, x)
                y0_distance, y_distance = 0, 0 # How much the drone is moving in Y direction
                x0_distance, x_distance = 0, 0 # How much the drone is moving in X direction
                dy, dx, dt = 0, 0, 0
                t_0 = time.time()
                counter = 0
                direction = 'up'
                print('I am (FIRST) at [y, x]: (%s, %s)' % (current_y, current_x))
                map_occupancy_grid[current_y, current_x] = 230

                while keep_flying: # and counter < 80:
                    
                    # Defines whether to move far from or close to the wall dependind 
                    # on the direction of the traffic (clockwise, counterclockwise).
                    if right_traffic:
                        traffic_flow_multi_ranger = multi_ranger.right
                        # print('Right traffic', multi_ranger.right)
                        vel_increment_y = -VELOCITY
                        OTHER_TRAFFIC_DISTANCE = 0
                    else:
                        traffic_flow_multi_ranger = multi_ranger.left
                        # print('<--<-<-<-<-Left traffic', multi_ranger.left)
                        vel_increment_y = VELOCITY
                        OTHER_TRAFFIC_DISTANCE = DISTANCE_TO_WALL * 2

                    print('mranger', traffic_flow_multi_ranger)

                    TOTAL_DISTANCE = DISTANCE_TO_WALL + OTHER_TRAFFIC_DISTANCE

                    # Define initial speed
                    velocity_x = 0.0
                    velocity_y = 0.0
                    obstacle_front = False
                    print('front: ', multi_ranger.front)
                    print('back: ', multi_ranger.back)

                    # If object in front of the forward-pointing range sensor move backwards
                    if is_close_to_obstacle(multi_ranger.front):
                        print('Obstacle in front detected at distance: ', multi_ranger.front)
                        velocity_x -= VELOCITY
                        obstacle_front = True
                    
                    # If object in front of the backward-pointing range sensor move forward
                    if is_close_to_obstacle(multi_ranger.back):
                        print('back obstacle')
                        velocity_x += VELOCITY

                    # If hand in front of the left-pointing range sensor move to the right
                    if is_close_to_obstacle(multi_ranger.left):
                        velocity_y -= VELOCITY
                    
                    # If hand in front of the right-pointing range sensor move to the left
                    if is_close_to_obstacle(multi_ranger.right):
                        velocity_y += VELOCITY
                    
                    # If hand in front of the right-pointing range sensor move to the left
                    print('Moving closer to wall: ', TOTAL_DISTANCE, 'm')

                    if traffic_flow_multi_ranger is not None:
                        SLACK = 0.05
                        print('Current distance: ', TOTAL_DISTANCE - SLACK)
                        if traffic_flow_multi_ranger > TOTAL_DISTANCE - SLACK:
                            print('Moving closer to wall: ', TOTAL_DISTANCE, 'm')
                            velocity_y += vel_increment_y

                        if traffic_flow_multi_ranger < TOTAL_DISTANCE + SLACK:
                            print('Moving far from wall: ', TOTAL_DISTANCE, 'm')
                            velocity_y -= vel_increment_y

                        # If object to close to the upward pointing ranging sensor land
                        if is_close_to_obstacle(multi_ranger.up):
                            print('stop?')
                            keep_flying = False

                        # Perform the motion defined above
                        # -------------------------------------
                        # Motion #2: Moving forward
                        # -------------------------------------
                        final_velocity_x = velocity_x if obstacle_front else forward_velocity
                        print('Moving forward at velocity of ', final_velocity_x)
                        if obstacle_front:
                            # keep_flying = False
                            # print('here?')
                            # if TRAFFIC_TYPE == RHT:
                            #     motion_commander.turn_left(90)
                            # else:
                            #     motion_commander.turn_right(90)
                            time.sleep(2)
                        else:
                            # pass
                            print('current vel x: ', final_velocity_x)
                            print('current vel y: ', velocity_y)
                            motion_commander.start_linear_motion(final_velocity_x, velocity_y, 0)

                        # Let's compute the current distance of the drone
                        t = time.time()
                        dt = t - t_0
                        t_0 = t
                        # Compute delta x: dx = vx.dt
                        if final_velocity_x < 0:
                            final_velocity_x = 0
                        dx = final_velocity_x * dt
                        # Compute delta y: dy = vy.dt
                        dy = velocity_y * dt
                        x_distance += dx
                        y_distance += dy
                        # print('Distance in X meters:', x_distance)
                        # print('Distance in Y meters:', y_distance)
                        
                        ## Correct rotation when shifting
                        # y0_distance = traffic_flow_multi_ranger
                        # x0_distance = multi_ranger.front
                        # if x_distance > (x0_distance + dx):

                        # For 0.1 seconds
                        # to try different frequency
                        time.sleep(0.1)
                        forward_cells = round((x_distance * 100) / 10)
                        if direction == 'up':
                            current_y += forward_cells
                        if direction == 'down':
                            current_y -= forward_cells
                        if direction == 'left':
                            current_x -= forward_cells
                        if direction == 'right':
                            current_x += forward_cells
                        print("I am in the Y: %s and X: %s" % (current_y, current_x))
                        last_y = current_y
                        last_x = current_x
                        print('I am at [y, x]: (%s, %s)' % (current_y, current_x))
                        if inside_boundaries(map_occupancy_grid, y=current_y, x=current_x):
                            map_occupancy_grid[last_y, last_x] = 100
                        else:
                            keep_flying = False
                    print('traffic flow ranger:', traffic_flow_multi_ranger)
                    counter += 1
    
    mapping.plot_map(map_occupancy_grid)
            
# Main program loop
if __name__ == '__main__':
    
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Call to perform drone mission
    perform_mision()
