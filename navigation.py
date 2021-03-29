# Packages (libraries) required
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Custom modules
#import initial_map as mapping
import update_map_2 as mapping
import server_client.asu_udp_client2 as uas_client

# Unique radio link identifier between computer and Craziflie UAS
URI = 'E7E7E7E7E9'
FULL_URI = 'radio://0/80/2M/%s' % (URI,)

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Control constants
VELOCITY = 0.3 # (m/s) Define speed increment
TOTAL_WIDTH = 2
DISTANCE_TO_WALL = TOTAL_WIDTH / 4
OTHER_TRAFFIC_DISTANCE = 0
SLACK = 0.05
keep_flying = True # Define a Boolean that goes to False to stop the mission
obstacle_front = False
obstacle_side = False
TOTAL_DISTANCE = 0
Map = mapping.UpdateMap()
current_section = ''

def create_trajectory(starting_point, goal_point, CLOCKWISE):

    if not CLOCKWISE:
        A_point = [Map.A[0][0] + (Map.A[2][0] - Map.A[0][0])/4, (Map.A[1][1] - Map.A[0][1])/2]
        B_point = [Map.B[0][0] + (Map.B[2][0] - Map.B[0][0])/4, Map.B[0][1] + 3*(Map.B[1][1] - Map.B[0][1])/4]
        C_point = [Map.C[0][0] + 3*(Map.C[2][0] - Map.C[0][0])/4, Map.C[0][1] + 3*(Map.C[1][1] - Map.C[0][1])/4]
        D_point = [Map.D[0][0] + 3*(Map.D[2][0] - Map.D[0][0])/4, (Map.D[1][1] - Map.D[0][1])/2]
        MISSIONS = {"D": D_point, "C": C_point, "B": B_point, "A": A_point}
    else:
        A_point = [Map.A[0][0] + 3*(Map.A[2][0] - Map.A[0][0])/4, (Map.A[1][1] - Map.A[0][1])/2]
        B_point = [Map.B[0][0] + 3*(Map.B[2][0] - Map.B[0][0])/4, Map.B[0][1] + (Map.B[1][1] - Map.B[0][1])/4]
        C_point = [Map.C[0][0] + (Map.C[2][0] - Map.C[0][0])/4, Map.C[0][1] + (Map.C[1][1] - Map.C[0][1])/4]
        D_point = [Map.D[0][0] + (Map.D[2][0] - Map.D[0][0])/4, (Map.D[1][1] - Map.D[0][1])/2]
        MISSIONS = {"A": A_point, "B":B_point, "C":C_point, "D":D_point}

    trajectory = [i for i in list(MISSIONS.values())[list(MISSIONS.keys()).index(starting_point):list(MISSIONS.keys()).index(goal_point)+1]]
    print("trajectory is ", trajectory)
    return trajectory

# Function that checks if the 'range' value is smaller than 0.2. 
# It return a Boolean True is so, and False if not
def is_close_to_obstacle(range):
    MAX_LIMIT = 0.4  # m
    MIN_LIMIT = 0.2 # m
    if range is None:
        return False
    else:
        return range > MIN_LIMIT and range < MAX_LIMIT

def inside_boundaries(occupancy_grid, y=0, x=0):
    grid_size = occupancy_grid.shape
    return y in range(grid_size[0]-4) and x in range(grid_size[1]-5)


def obstacle_avoidance(multi_ranger, velocity_x, velocity_y):

    global VELOCITY
    global obstacle_front
    global obstacle_side

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
    # on the direction of the traffic (clockwise or not).
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
    global keep_flying
    # If object to close to the upward pointing ranging sensor land
    if is_close_to_obstacle(up_ranger):
        print('stop?')
        keep_flying = False

def perform_mision(trajectory, clockwise, starting_point):
    
    global DISTANCE_TO_WALL
    global OTHER_TRAFFIC_DISTANCE
    global TRAFFIC_TYPE
    global keep_flying
    global obstacle_front
    global obstacle_side
    global TOTAL_DISTANCE
    global counter_checkpoint
    global URI
    global FULL_URI
    global current_section

    TRAFFIC_TYPE = clockwise
    right_traffic = not TRAFFIC_TYPE
    occupancy_grid = np.zeros((78, 116))
    last_y, last_x = 0, 0
    counter_checkpoint = 1
    y_check = trajectory[0][1]
    x_check = trajectory[0][0]
    uas_traffic_conflict = False
    current_section = starting_point
    measure_ranger = False

    # Create a synchronous connection with the UAS, UAS is now known as 'scf'
    with SyncCrazyflie(FULL_URI) as scf:
        height = 0.2
        # Create a motion commandor (tool that let's you change the motion of the UAS) for 'scf'
        with MotionCommander(scf, height) as motion_commander:
            
            # Take off
            print('Taking off! -by default up to 0.3m-')
            
            print('TRAFFIC TYPE: ', TRAFFIC_TYPE, 
                'Clockwise' if TRAFFIC_TYPE else 'Counterclockwise')
            
            # Wait a bit (i.e. 5 seconds) before doing the next step
            time.sleep(5)

            # Define a multirange object 'multi-ranger'
            with Multiranger(scf) as multi_ranger:
                
                # Continue performing the loop while 'keep_flying' Booelan is True
                forward_velocity = 0.3
                
                initial_y, initial_x = 15, 20 # Initial position in the map (y, x)
                y_global_distance, x_global_distance = initial_y, initial_x # Current position in the map (y, x)
                y0_drone_distance, y_drone_distance = 0, 0 # How much the drone is moving in Y direction
                x0_drone_distance, x_drone_distance = 0, 0 # How much the drone is moving in X direction
                y_drone_distance_temp, x_drone_distance_temp = 0, 0
                dy, dx, dt = 0, 0, 0
                t_0 = time.time()
                counter = 0
                # CHECK THIS
                direction = 'up'
                print('I am (FIRST) at [y, x]: (%s, %s)' % (y_global_distance, x_global_distance))
                occupancy_grid[y_global_distance, x_global_distance] = 230

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
                initialize_coords = True
                checkpoint_reached = False

                while keep_flying:
                    
                    Map.Update_map(occupancy_grid_upd, Occ_Map)
                    obstacle_front = False
                    obstacle_side = False
                    # Define initial speed
                    velocity_x = 0.0
                    velocity_y = 0.0
                    velocity_wall = 0.0
                    print('front: ', multi_ranger.front)
                    print('back: ', multi_ranger.back)
                    # Define the multi ranger sensor for wall following
                    velocity_x, velocity_y = obstacle_avoidance(multi_ranger, velocity_x, velocity_y)
                    if right_traffic:
                        # Counterclockwise
                        print('right traff')
                        traffic_flow_multi_ranger = multi_ranger.right
                        velocity_wall = -VELOCITY
                        OTHER_TRAFFIC_DISTANCE = 0
                    else:
                        # Clockwise
                        traffic_flow_multi_ranger = multi_ranger.left
                        velocity_wall = VELOCITY
                        OTHER_TRAFFIC_DISTANCE = DISTANCE_TO_WALL * 2

                    print('mranger', traffic_flow_multi_ranger)

                    # If Clockwise, TOTAL_DISTANCE = 0.5 + 1 = 1.5
                    # If Counterclockwise, TOTAL_DISTANCE = 0.5 + 0 = 0.5
                    TOTAL_DISTANCE = DISTANCE_TO_WALL + OTHER_TRAFFIC_DISTANCE

                    # Check this. too far for CLOCKWISE
                    if traffic_flow_multi_ranger is not None:

                        velocity_wall = wall_following(traffic_flow_multi_ranger, velocity_wall)
                        stop_flying(multi_ranger.up)

                        # Define x and y velocity
                        final_velocity_x = velocity_x if obstacle_front else forward_velocity
                        final_velocity_y = velocity_y if obstacle_side else velocity_wall

                        if uas_traffic_conflict:
                            final_velocity_x = 0
                            final_velocity_y = 0

                        print('Moving forward at velocity of ', final_velocity_x)

                        

                        # PLANNING
                        print("checkpoint is ", counter_checkpoint)
                        print("trajectory[counter_checkpoint] is ", trajectory[counter_checkpoint])
                        if counter_checkpoint < len(trajectory):

                            if initialize_coords:
                                # SET DIRECTION
                                iter_y_check = trajectory[counter_checkpoint][1]
                                iter_x_check = trajectory[counter_checkpoint][0]
                                # Check if direction is up or down
                                if y_check != iter_y_check:
                                    direction = 'up' if iter_y_check > y_check else 'down'
                                # Check if direction is left or right
                                elif x_check != iter_x_check:
                                    direction = 'right' if iter_x_check > x_check else 'left'
                                y_check = iter_y_check
                                x_check = iter_x_check
                                if right_traffic:
                                    if direction == 'up':
                                        x_global_distance = float(occupancy_grid.shape[1]-round(multi_ranger.right*10))
                                        y_global_distance = float(multi_ranger.back*10)
                                    elif direction == 'down':
                                        x_global_distance = float(round(multi_ranger.right*10))
                                        y_global_distance = float(occupancy_grid.shape[0]-round(multi_ranger.back*10))
                                    elif direction == 'left':
                                        x_global_distance = float(occupancy_grid.shape[1]-round(multi_ranger.back*10))
                                        y_global_distance = float(occupancy_grid.shape[0]-round(multi_ranger.right*10))
                                    #x_global_distance = float(occupancy_grid.shape[1]-round(multi_ranger.right*10))
                                else:
                                    if direction == 'up':
                                        x_global_distance = float(multi_ranger.left*10)
                                        y_global_distance = float(multi_ranger.back*10)
                                    elif direction == 'down':
                                        x_global_distance = float(occupancy_grid.shape[1]-round(multi_ranger.left*10))
                                        y_global_distance = float(occupancy_grid.shape[0]-round(multi_ranger.back*10))
                                    elif direction == 'right':
                                        x_global_distance = float(multi_ranger.back*10)
                                        y_global_distance = float(occupancy_grid.shape[0]-round(multi_ranger.left*10))
                                    #x_global_distance = float(multi_ranger.left*10)
                                #y_global_distance = float(multi_ranger.back*10)
                                initial_y = y_global_distance
                                initial_x = x_global_distance
                                print("direction is ", direction)
                                print("initial_y ", initial_y)
                                print("initial_x ", initial_x)
                                initialize_coords = False
                                initialized_x_y = False
                                #if not counter_checkpoint == 1:
                                #    checkpoint_reached = True


                            # LOCALIZATION
                            # Let's compute the current distance of the drone
                            #if counter_time > 1:
                            t = time.time()
                            #counter_time = 0
                            #t = time.time()
                            dt = t - t_0
                            t_0 = t  
                            #counter_time+=1
                            # Compute delta x: dx = vx.dt
                            if final_velocity_x < 0:
                                final_velocity_x = 0
                            dx = math.sqrt(final_velocity_x**2 + final_velocity_y**2) * math.cos(math.atan2(final_velocity_y,final_velocity_x)) * dt
                            # Compute delta y: dy = vy.dt
                            dy = velocity_y * dt
                            x_drone_distance += dx*10
                            y_drone_distance += dy*10

                            print('Distance in X meters:', x_drone_distance/10)
                            # print('Distance in Y meters:', y_drone_distance)

                            ## Correct rotation when shifting
                            # y0_distance = traffic_flow_multi_ranger
                            # x0_distance = multi_ranger.front
                            # if x_distance > (x0_distance + dx):
                            
                            print('->->-> Forward cells', x_drone_distance)
                            if right_traffic:
                                if direction == 'up':
                                    y_global_distance = initial_y + x_drone_distance
                                    x_global_distance = occupancy_grid.shape[1] - traffic_flow_multi_ranger*10
                                elif direction == 'down':
                                    y_global_distance = initial_y - x_drone_distance
                                    x_global_distance = traffic_flow_multi_ranger*10
                                elif direction == 'left':
                                    x_global_distance = initial_x - x_drone_distance
                                    y_global_distance = occupancy_grid.shape[0] - traffic_flow_multi_ranger*10
                                elif direction == 'right':
                                    x_global_distance = initial_x + x_drone_distance
                                    y_global_distance = occupancy_grid.shape[0] - traffic_flow_multi_ranger*10
                            else:
                                if direction == 'up':
                                    y_global_distance = initial_y + x_drone_distance
                                    x_global_distance = traffic_flow_multi_ranger*10
                                elif direction == 'down':
                                    y_global_distance = initial_y - x_drone_distance
                                    x_global_distance = occupancy_grid.shape[1] - traffic_flow_multi_ranger*10
                                elif direction == 'left':
                                    x_global_distance = initial_x - x_drone_distance
                                    y_global_distance = occupancy_grid.shape[0] - traffic_flow_multi_ranger*10
                                elif direction == 'right':
                                    x_global_distance = initial_x + x_drone_distance
                                    y_global_distance = occupancy_grid.shape[0] - traffic_flow_multi_ranger*10
                            y_global_distance = round(y_global_distance)
                            x_global_distance = round(x_global_distance)
                            print('I am in the Y: %s and X: %s' % (y_global_distance, x_global_distance))
                            print('I am at [y, x]: (%s, %s)' % (y_global_distance, x_global_distance))
                            print("x_global_distance-trajectory[counter_checkpoint][0]/10 ", (x_global_distance-trajectory[counter_checkpoint][0]/10)**2)
                            print("y_global_distance-trajectory[counter_checkpoint][1]/10 ", (y_global_distance-trajectory[counter_checkpoint][1]/10)**2)
                            if inside_boundaries(occupancy_grid, y=y_global_distance, x=x_global_distance):
                                occupancy_grid_upd[last_y, last_x] = 100
                            else:
                                print('fuck you')
                                #keep_flying = False

                            # SEND THE SECTION:
                            if right_traffic:
                                if x_global_distance>=Map.A[0][0]/10 and x_global_distance<=Map.A[2][0]/10 and y_global_distance>=Map.A[0][1]/10 and y_global_distance<=Map.B[1][1]/10:
                                    current_section = "BA"
                                if x_global_distance>=2/3*Map.B[1][0]/10 and x_global_distance<=Map.C[2][0]/10 and y_global_distance>=Map.B[2][1]/10 and y_global_distance<=Map.B[3][1]/10:
                                    current_section = "CB3"
                                if x_global_distance>=1/3*Map.B[1][0]/10 and x_global_distance<=2/3*Map.C[2][0]/10 and y_global_distance>=Map.B[2][1]/10 and y_global_distance<=Map.B[3][1]/10:
                                    current_section = "CB2"
                                if x_global_distance>=Map.B[1][0]/10 and x_global_distance<=1/3*Map.C[2][0]/10 and y_global_distance>=Map.B[2][1]/10 and y_global_distance<=Map.B[3][1]/10:
                                    if direction == "left":
                                        current_section = "CB1"
                                    else:
                                        current_section = "BA"
                                if x_global_distance>=Map.C[0][0]/10 and x_global_distance<=Map.C[2][0]/10 and y_global_distance>=Map.D[0][1]/10 and y_global_distance<=Map.C[1][1]/10:
                                    if direction == "up":
                                        current_section = "DC"
                                    else:
                                        current_section = "CB3"
                            else:
                                if x_global_distance>=Map.A[0][0]/10 and x_global_distance<=Map.A[2][0]/10 and y_global_distance>=Map.A[0][1]/10 and y_global_distance<=Map.B[1][1]/10:
                                    current_section = "AB"
                                if x_global_distance>=2/3*Map.B[1][0]/10 and x_global_distance<=Map.C[2][0]/10 and y_global_distance>=Map.B[2][1]/10 and y_global_distance<=Map.B[3][1]/10:
                                    current_section = "BC3"
                                if x_global_distance>=1/3*Map.B[1][0]/10 and x_global_distance<=2/3*Map.C[2][0]/10 and y_global_distance>=Map.B[2][1]/10 and y_global_distance<=Map.B[3][1]/10:
                                    current_section = "BC2"
                                if x_global_distance>=Map.B[1][0]/10 and x_global_distance<=1/3*Map.C[2][0]/10 and y_global_distance>=Map.B[2][1]/10 and y_global_distance<=Map.B[3][1]/10:
                                    if direction == "right":
                                        current_section = "BC1"
                                    else:
                                        current_section = "AB"
                                if x_global_distance>=Map.C[0][0]/10 and x_global_distance<=Map.C[2][0]/10 and y_global_distance>=Map.D[0][1]/10 and y_global_distance<=Map.C[1][1]/10:
                                    if direction == "down":
                                        current_section = "CD"
                                    else:
                                        current_section = "BC3"

                            # If we are going to start the navigation or we are 
                            # changing section, we inform the server.
                            print('***Talking to the server...***')
                            #uas_traffic_conflict = uas_client.main(URI, current_section, height)
                            
                            # CHECK IF REACHED CHECKPOINT
                            print("y_global_distance ", y_global_distance)
                            print("trajectory[counter_checkpoint][1]/10 ", trajectory[counter_checkpoint][1]/10)
                            if (direction == 'up' or direction == 'down') and abs(y_global_distance-trajectory[counter_checkpoint][1]/10) < 4:
                                print("multi_ranger.front is ", multi_ranger.front)
                                measure_ranger = True
                            elif (direction == 'left' or direction == 'right') and abs(x_global_distance-trajectory[counter_checkpoint][0]/10) < 4:
                                print('Reached the next checkpoint by mapping (Horizontal)')
                                measure_ranger = True
                                #if right_traffic:
                                #    if multi_ranger.front <= 0.5:
                                        
                                #        checkpoint_reached = True
                                #else:
                                #    if multi_ranger.front <= 1.5:
                                #        print('Reached the next checkpoint by mapping (Horizontal)')
                                #        checkpoint_reached = True
                            if measure_ranger:
                                if right_traffic:
                                    if multi_ranger.front <= 0.5:
                                        print('Reached the next checkpoint by mapping (Vertical)')
                                        checkpoint_reached = True
                                else:
                                    if multi_ranger.front <= 1.5:
                                        print('Reached the next checkpoint by mapping (Vertical)')
                                        checkpoint_reached = True
                            if checkpoint_reached:
                                if right_traffic:
                                    print('the turn left')
                                    motion_commander.turn_left(90)
                                else:
                                    print('the turn right')
                                    motion_commander.turn_right(90)
                                counter_checkpoint += 1
                                x_drone_distance = 0
                                y_drone_distance = 0
                                initialize_coords = True
                                checkpoint_reached = False
                                time.sleep(4)
                                t_0 = time.time()
                                measure_ranger = False
                            elif not uas_traffic_conflict:
                                print('current vel x: ', final_velocity_x)
                                print('current vel y: ', final_velocity_y)
                                print('IT GOES TO MOVE')
                                motion_commander.start_linear_motion(final_velocity_x, final_velocity_y, 0)
                            
                            


                        else:
                            keep_flying = False
                        #print('keep_flying', keep_flying)
                    print('traffic flow ranger:', traffic_flow_multi_ranger)
                    
                    #mapping.plot_grid(occupancy_grid)
    
    #mapping.plot_map(map_occupancy_grid)
            
# Main program loop
if __name__ == '__main__':
    
    CLOCKWISE = True
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Fake client registrations to check conflict:
    URI2 = 'E7E7E7E7E7'
    URI3 = 'E7E7E7E7E8'
    uas_client.main(URI2, 'BA', '0.3')
    uas_client.main(URI3, 'BC', '0.3')

    MISSIONS = [('A', 'C')] # ('A', 'C'), ('C', 'B'), ('B', 'D')
    for mission in MISSIONS:
        starting_point = mission[0]
        goal_point = mission[1]
        if starting_point > goal_point:
            CLOCKWISE = False # True: left to right, False: right to left
        else:
            CLOCKWISE = True
        trajectory = create_trajectory(starting_point, goal_point, CLOCKWISE)
        # Call to perform drone mission
        perform_mision(trajectory, CLOCKWISE, starting_point)
