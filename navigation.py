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
CLOCKWISE = True # True: left to right, False: right to left
TRAFFIC_TYPE = CLOCKWISE
VELOCITY = 0.1 # (m/s) Define speed increment
TOTAL_WIDTH = 2
DISTANCE_TO_WALL = TOTAL_WIDTH / 4
OTHER_TRAFFIC_DISTANCE = 0
SLACK = 0.02
keep_flying = True # Define a Boolean that goes to False to stop the mission
obstacle_front = False
obstacle_side = False
TOTAL_DISTANCE = 0

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
    return y in range(grid_size[0]-10) and x in range(grid_size[1])


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

def perform_mision():
    
    global DISTANCE_TO_WALL
    global OTHER_TRAFFIC_DISTANCE
    global TRAFFIC_TYPE
    global CLOCKWISE
    global keep_flying
    global obstacle_front
    global obstacle_side
    global TOTAL_DISTANCE

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
            
            right_traffic = not TRAFFIC_TYPE
            print('TRAFFIC TYPE: ', TRAFFIC_TYPE, 
                'Clockwise' if TRAFFIC_TYPE else 'Counterclockwise')
            # Wait a bit (i.e. 5 seconds) before doing the next step
            time.sleep(5)

            # Define a multirange object 'multi-ranger'
            with Multiranger(scf) as multi_ranger:
                
                # Continue performing the loop while 'keep_flying' Booelan is True
                forward_velocity = 0.2
                
                initial_y, initial_x = 15, 20 # Initial position in the map (y, x)
                current_y, current_x = initial_y, initial_x # Current position in the map (y, x)
                y0_distance, y_distance = 0, 0 # How much the drone is moving in Y direction
                x0_distance, x_distance = 0, 0 # How much the drone is moving in X direction
                dy, dx, dt = 0, 0, 0
                t_0 = time.time()
                counter = 0
                direction = 'up'
                print('I am (FIRST) at [y, x]: (%s, %s)' % (current_y, current_x))
                map_occupancy_grid[current_y, current_x] = 230

                while keep_flying:
                    print('Counter', counter)
                    obstacle_front = False
                    obstacle_side = False
                    # Define initial speed
                    velocity_x = 0.0
                    velocity_y = 0.0
                    velocity_wall = 0.0
                    
                    print('front: ', multi_ranger.front)
                    print('back: ', multi_ranger.back)

                    velocity_x, velocity_y = obstacle_avoidance(multi_ranger, velocity_x, velocity_y)

                    # Define the multi ranger sensor for wall following
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

                        # Perform the motion defined above
                        # -------------------------------------
                        # Motion #2: Moving forward
                        # -------------------------------------
                        final_velocity_x = velocity_x if obstacle_front else forward_velocity
                        final_velocity_y = velocity_y if obstacle_side else velocity_wall

                        print('Moving forward at velocity of ', final_velocity_x)
                        if obstacle_front:
                            # keep_flying = False
                            # print('here?')
                            # if right_traffic:
                            #     motion_commander.turn_left(90)
                            # else:
                            #     motion_commander.turn_right(90)
                            # time.sleep(2)
                            pass
                        else:
                            # pass
                            print('current vel x: ', final_velocity_x)
                            print('current vel y: ', final_velocity_y)
                            motion_commander.start_linear_motion(final_velocity_x, final_velocity_y, 0)

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
                        forward_cells = round((x_distance * 100) / 10)
                        print('->->-> Forward cells', forward_cells)
                        if direction == 'up':
                            print('++++++++yes UP', forward_cells)
                            current_y = initial_y + forward_cells
                        if direction == 'down':
                            current_y = initial_y - forward_cells
                        if direction == 'left':
                            current_x = initial_x - forward_cells
                        if direction == 'right':
                            current_x = initial_x + forward_cells
                        print('I am in the Y: %s and X: %s' % (current_y, current_x))
                        print('I am at [y, x]: (%s, %s)' % (current_y, current_x))
                        # if current_y < 50 and inside_boundaries(map_occupancy_grid, y=current_y, x=current_x):
                        if inside_boundaries(map_occupancy_grid, y=current_y, x=current_x):
                            map_occupancy_grid[current_y, current_x] = 100
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
