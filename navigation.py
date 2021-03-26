# Packages (libraries) required
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Custom modules
#import initial_map as mapping
import update_map_2 as mapping

# Unique radio link identifier between computer and Craziflie UAS
URI = 'radio://0/80/2M/E7E7E7E7E9'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Control constants
COUNTERCLOCKWISE = 0 # Going
CLOCKWISE = 1 # Returning
TRAFFIC_TYPE = CLOCKWISE
VELOCITY = 0.2 # (m/s) Define speed increment
TOTAL_WIDTH = 2
DISTANCE_TO_WALL = TOTAL_WIDTH / 4
OTHER_TRAFFIC_DISTANCE = 0
SLACK = 0.1
keep_flying = True # Define a Boolean that goes to False to stop the mission
obstacle_front = False
obstacle_side = False
TOTAL_DISTANCE = 0
starting_point = "A"
goal_point = "C"
Map = mapping.UpdateMap()

if COUNTERCLOCKWISE:
    A_point = [Map.A[0][0] + (Map.A[2][0] - Map.A[0][0])/4, (Map.A[1][1] - Map.A[0][1])/2]
    B_point = [Map.B[0][0] + (Map.B[2][0] - Map.B[0][0])/4, Map.B[0][1] + 3*(Map.B[1][1] - Map.B[0][1])/4]
    C_point = [Map.C[0][0] + 3*(Map.C[2][0] - Map.C[0][0])/4, Map.C[0][1] + 3*(Map.C[1][1] - Map.C[0][1])/4]
    D_point = [Map.D[0][0] + 3*(Map.D[2][0] - Map.D[0][0])/4, (Map.D[1][1] - Map.D[0][1])/2]
    MISSIONS = {"D":D_point, "C":C_point, "B":B_point, "A": A_point}
else:
    A_point = [Map.A[0][0] + 3*(Map.A[2][0] - Map.A[0][0])/4, (Map.A[1][1] - Map.A[0][1])/2]
    B_point = [Map.B[0][0] + 3*(Map.B[2][0] - Map.B[0][0])/4, Map.B[0][1] + (Map.B[1][1] - Map.B[0][1])/4]
    C_point = [Map.C[0][0] + (Map.C[2][0] - Map.C[0][0])/4, Map.C[0][1] + (Map.C[1][1] - Map.C[0][1])/4]
    D_point = [Map.D[0][0] + (Map.D[2][0] - Map.D[0][0])/4, (Map.D[1][1] - Map.D[0][1])/2]
    MISSIONS = {"A": A_point, "B":B_point, "C":C_point, "D":D_point}

trajectory = [i for i in list(MISSIONS.values())[list(MISSIONS.keys()).index(starting_point):list(MISSIONS.keys()).index(goal_point)+1]]
counter_checkpoint = 0
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
    #if y in range(grid_size[0]-2) and x in range(grid_size[1]):
    #    return True
    #else: 
    #    return False
    return y in range(grid_size[0]-10) and x in range(grid_size[1])


def obstacle_avoidance(multi_ranger, velocity_x, velocity_y):

    global VELOCITY

    # If object in front of the forward-pointing range sensor move backwards
    if is_close_to_obstacle(multi_ranger.front):
        print('Obstacle in front detected at distance: ', multi_ranger.front)
        velocity_x = -VELOCITY
        obstacle_front = True
                    
    # If object in front of the backward-pointing range sensor move forward
    if is_close_to_obstacle(multi_ranger.back):
        print('back obstacle')
        velocity_x = VELOCITY

    # If hand in front of the left-pointing range sensor move to the right
    if is_close_to_obstacle(multi_ranger.left):
        print('Left obstacle')
        velocity_y = -VELOCITY
        obstacle_side = True
                    
    # If hand in front of the right-pointing range sensor move to the left
    if is_close_to_obstacle(multi_ranger.right):
        print('Right obstacle')
        velocity_y = VELOCITY
        obstacle_side = True

    return (velocity_x, velocity_y)

def wall_following(traffic_flow_multi_ranger, velocity_wall):

    global TOTAL_DISTANCE, SLACK
    # Defines whether to move far from or close to the wall dependind 
    # on the direction of the traffic (CLOCKWISE, counterCLOCKWISE).                  
    print('Control distance: ', TOTAL_DISTANCE - SLACK)
    print('and traffic_flow_multi_ranger: ', traffic_flow_multi_ranger)
    if traffic_flow_multi_ranger > TOTAL_DISTANCE + SLACK:
        print('Moving closer to wall: ', TOTAL_DISTANCE, 'm')
        return velocity_wall

    if traffic_flow_multi_ranger < TOTAL_DISTANCE - SLACK:
        print('Moving far from wall: ', TOTAL_DISTANCE, 'm')
        return -velocity_wall
    return 0

def stop_flying(up_ranger):
    # If object to close to the upward pointing ranging sensor land
    if is_close_to_obstacle(up_ranger):
        print('stop?')
        keep_flying = False

def perform_mision():
    
    global DISTANCE_TO_WALL
    global OTHER_TRAFFIC_DISTANCE
    global TRAFFIC_TYPE
    global COUNTERCLOCKWISE, CLOCKWISE
    global keep_flying
    global obstacle_front
    global obstacle_side
    global TOTAL_DISTANCE
    global counter_checkpoint

    occupancy_grid = np.zeros((80, 120))
    #occupancy_grid = Map.initialize_map(occupancy_grid)
    last_y, last_x = 0, 0
    # current_y, current_x = 0, 20
    Initialized_x_y = True

    # Create a synchronous connection with the UAS, UAS is now known as 'scf'
    with SyncCrazyflie(URI) as scf:
        
        # Create a motion commandor (tools that let's you change the motion of the UAS) for 'scf'
        with MotionCommander(scf, 0.2) as motion_commander:
            
            # Take off
            print('Taking off! -by default up to 0.3m-')
            right_traffic = TRAFFIC_TYPE == 0
            print('TRAFFIC TYPE', TRAFFIC_TYPE)
            # Wait a bit (i.e. 5 seconds) before doing the next step
            time.sleep(5)

            # Define a multirange object 'multi-ranger'
            with Multiranger(scf) as multi_ranger:
                
                # Continue performing the loop while 'keep_flying' Booelan is True
                forward_velocity = 0.2
                
                current_y, current_x = 15, 20 # Current position in the map (y, x)


                dy, dx, dt = 0, 0, 0
                t_0 = time.time()
                counter = 0
                direction = 'up'
                print('I am (FIRST) at [y, x]: (%s, %s)' % (current_y, current_x))
                occupancy_grid[current_y, current_x] = 230

                # ANIMATE OCCUPANCY GRID
                plt.ion() 
                matplotlib.rc('xtick', labelsize=5)
                matplotlib.rc('ytick', labelsize=5)
                plt.figure(figsize=(10.0, 11.5))
                plt.title("Drone map")
                occupancy_grid_upd = Map.initialize_map(occupancy_grid)
                Occ_Map = plt.imshow(occupancy_grid_upd, cmap="gray")
                plt.gca().invert_yaxis()
                plt.grid(linestyle = '--', linewidth = 0.2)

                while keep_flying: # and counter < 80:
                    Map.Update_map(occupancy_grid_upd, Occ_Map)
                    obstacle_front = False
                    obstacle_side = False
                    # Define initial speed
                    velocity_x = 0.0
                    velocity_y = 0.0
                    velocity_wall = 0.0
                    print('front: ', multi_ranger.front)
                    print('back: ', multi_ranger.back)

                    #velocity_x, velocity_y = obstacle_avoidance(multi_ranger, velocity_x, velocity_y)

                    # Define the multi ranger sensor for wall following
                    if right_traffic:
                        # CounterCLOCKWISE
                        print('right traff')
                        traffic_flow_multi_ranger = multi_ranger.right
                        velocity_wall = -VELOCITY
                        OTHER_TRAFFIC_DISTANCE = 0
                    else:
                        # CLOCKWISE
                        traffic_flow_multi_ranger = multi_ranger.left
                        velocity_wall = VELOCITY
                        OTHER_TRAFFIC_DISTANCE = DISTANCE_TO_WALL * 2

                    print('mranger', traffic_flow_multi_ranger)

                    # If CLOCKWISE, TOTAL_DISTANCE = 0.5 + 1 = 1.5
                    # If CounterCLOCKWISE, TOTAL_DISTANCE = 0.5 + 0 = 0.5
                    TOTAL_DISTANCE = DISTANCE_TO_WALL + OTHER_TRAFFIC_DISTANCE

                    # Check this. too far for CLOCKWISE
                    if traffic_flow_multi_ranger is not None:
                        if Initialized_x_y:
                            if COUNTERCLOCKWISE:
                                y_distance = float(occupancy_grid.shape[0]-round(multi_ranger.right*10))
                            else:
                                y_distance = float(multi_ranger.left*10)
                            x_distance = float(multi_ranger.back*10)
                            Initialized_x_y = False

                        velocity_wall = wall_following(traffic_flow_multi_ranger, velocity_wall)
                        stop_flying(multi_ranger.up)

                        # Define x and y velocity
                        final_velocity_x = velocity_x if obstacle_front else forward_velocity
                        final_velocity_y = velocity_y if obstacle_side else velocity_wall

                        print('Moving forward at velocity of ', final_velocity_x)




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
                        x_distance += dx*10
                        y_distance += dy*10
                        print('Distance in X meters:', x_distance)
                        # print('Distance in Y meters:', y_distance)

                        ## Correct rotation when shifting
                        # y0_distance = traffic_flow_multi_ranger
                        # x0_distance = multi_ranger.front
                        # if x_distance > (x0_distance + dx):

                        # For 0.1 seconds
                        # to try different frequency
                        time.sleep(0.1)
                        # Check this. it goes too fast
                        forward_cells = round((dx * 100) / 10)
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
                        if inside_boundaries(occupancy_grid, y=current_y, x=current_x):
                            occupancy_grid[last_y, last_x] = 100
                        else:
                            keep_flying = False

                        # PLANNING
                        if counter_checkpoint < len(trajectory):
                            if ((x_distance-trajectory[counter_checkpoint][0]/10)**2 + (y_distance-trajectory[counter_checkpoint][1]/10)**2 < 1):
                                print('Reached the next checkpoint by mapping')
                                counter_checkpoint+=1
                                if (TRAFFIC_TYPE == COUNTERCLOCKWISE):
                                    motion_commander.turn_left(90)
                                else:
                                    motion_commander.turn_right(90)
                            elif (x_distance-trajectory[counter_checkpoint][0]/10)**2 + (y_distance-trajectory[counter_checkpoint][1]/10)**2 < 3:
                                                                
                                if obstacle_front:
                                    counter_checkpoint+=1

                                    #keep_flying = False
                                    print('Reached the next checkpoint by ranger')
                                    if TRAFFIC_TYPE == COUNTERCLOCKWISE:
                                        motion_commander.turn_left(90)
                                    else:
                                        motion_commander.turn_right(90)
                                    time.sleep(2)
                                else:
                                    # pass
                                    print('current vel x: ', final_velocity_x)
                                    print('current vel y: ', final_velocity_y)
                                    motion_commander.start_linear_motion(final_velocity_x, final_velocity_y, 0)
                            else:
                                print('current vel x: ', final_velocity_x)
                                print('current vel y: ', final_velocity_y)
                                motion_commander.start_linear_motion(final_velocity_x, final_velocity_y, 0)
                        else:
                            keep_flying = False

                    print('traffic flow ranger:', traffic_flow_multi_ranger)
                    
                    counter += 1
                    #mapping.plot_grid(occupancy_grid)
    
    #mapping.plot_map(map_occupancy_grid)
            
# Main program loop
if __name__ == '__main__':
    
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Call to perform drone mission
    perform_mision()
