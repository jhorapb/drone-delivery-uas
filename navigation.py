# Packages (libraries) required
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Unique radio link identifier between computer and Craziflie UAS
URI = 'radio://0/80/2M/E7E7E7E7E9'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Control constants
RHT = 0 # Going
LHT = 1 # Returning
TRAFFIC_TYPE = RHT
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

def perform_mision():
    
    global DISTANCE_TO_WALL
    global OTHER_TRAFFIC_DISTANCE
    global TRAFFIC_TYPE
    global RHT, LHT

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
            
            # Wait a bit (i.e. 5 seconds) before doing the next step
            time.sleep(5)

            # Define a multirange object 'multi-ranger'
            with Multiranger(scf) as multi_ranger:
                
                # Define a Boolean that goes to False to stop the mission
                keep_flying = True

                right_traffic = TRAFFIC_TYPE == RHT
                if right_traffic:
                    print('what?', multi_ranger.right)
                    traffic_flow_multi_ranger = multi_ranger.right
                    vel_increment_y = VELOCITY
                    OTHER_TRAFFIC_DISTANCE = 0
                else:
                    traffic_flow_multi_ranger = multi_ranger.left
                    vel_increment_y = VELOCITY
                    OTHER_TRAFFIC_DISTANCE = DISTANCE_TO_WALL * 2

                print('mranger', traffic_flow_multi_ranger)

                TOTAL_DISTANCE = DISTANCE_TO_WALL + OTHER_TRAFFIC_DISTANCE
                # Continue performing the loop while 'keep_flying' Booelan is True
                forward_velocity = 0.5
                
                y_coord, x_coord = 0, 0 # Current position in the map (y, x)
                y0_distance, y_distance = 0, 0 # How much the drone is moving in Y direction
                x0_distance, x_distance = 0, 0 # How much the drone is moving in X direction
                dy, dx, dt = 0, 0, 0
                t_0 = time.time()
                while keep_flying:
                    # print(keep_flying)
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
                    # motion_commander.start_linear_motion(velocity_x, velocity_y, 0)
                    # -------------------------------------
                    # Motion #2: Moving forward
                    # -------------------------------------
                    final_velocity_x = velocity_x if obstacle_front else forward_velocity
                    print('Moving forward always at 0.5m/s velox ', final_velocity_x)
                    if obstacle_front:
                        # if TRAFFIC_TYPE == RHT:
                        #     motion_commander.turn_left(90)
                        # else:
                        #     motion_commander.turn_right(90)
                        time.sleep(2)
                    else:
                        # pass
                        motion_commander.start_linear_motion(final_velocity_x, velocity_y, 0)

                    # Let's compute the current distance of the drone
                    t = time.time()
                    dt = t - t_0
                    t_0 = t
                    # Compute delta x: dx = vx.dt
                    dx = final_velocity_x * dt
                    # Compute delta y: dy = vy.dt
                    dy = velocity_y * dt
                    x_distance += dx
                    y_distance += dy
                    print('Distance in X meters:', x_distance)
                    print('Distance in Y meters:', y_distance)
                    
                    ## Correct rotation when shifting
                    # y0_distance = traffic_flow_multi_ranger
                    # x0_distance = multi_ranger.front
                    # if x_distance > (x0_distance + dx):

                    # For 0.1 seconds
                    # to try different frequency
                    time.sleep(0.1)
                    x_coord = round((x_distance * 100) / 10)
                    print("I am in the X: %s and Y: %s" % (x_coord, y_coord))
            
# Main program loop
if __name__ == '__main__':
    
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Call to perform drone mission
    perform_mision()
