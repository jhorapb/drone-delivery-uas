"""
    Avionics Systems for UAS

    Example that shows the ranging sensors are used on the CrazyFlie
    Based on 'multirange.py'

    Copyright (c) 2020 - FF-ILR-TUBerlin
"""

# Packages (libraries) required
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

# Unique radio link identifier between computer and Craziflie UAS
# URI = 'radio://0/80/2M'
URI = 'radio://0/80/2M/E7E7E7E7E9'

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Function that checks if the 'range' value is smaller than 0.2. 
# It return a Boolean True is so, and False if not
def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

# Main program loop
if __name__ == '__main__':
    
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Define the CrazyFlie object
    cf = Crazyflie(rw_cache='./cache')

    # Define scope of a synchronous connection
    with SyncCrazyflie(URI, cf=cf) as scf:
 
        # We take off when the commander is created
        with MotionCommander(scf) as motion_commander:
        
            # Define a multirange object 'multi-ranger'
            with Multiranger(scf) as multi_ranger:
                
                # Define a Boolean that goes to False 
                # if the user put his hand close to thye upwards pointing ranging sensor
                keep_flying = True

                # Continue performing the loop while 'keep_flying' Booelan is True    
                while keep_flying:
                    print('right', multi_ranger.right)
                    print('left', multi_ranger.left)

                    # Define speed increment
                    VELOCITY = 0.5
                    
                    # Define initial speed
                    velocity_x = 0.0
                    velocity_y = 0.0

                    # If hand in front of the forward-pointing range sensor move backwards
                    if is_close(multi_ranger.front):
                        velocity_x -= VELOCITY
                    
                    # If hand in front of the backward-pointing range sensor move forward
                    if is_close(multi_ranger.back):
                        velocity_x += VELOCITY

                    # If hand in front of the left-pointing range sensor move to the right
                    if is_close(multi_ranger.left):
                        velocity_y -= VELOCITY
                    
                    # If hand in front of the right-pointing range sensor move to the left
                    if is_close(multi_ranger.right):
                        velocity_y += VELOCITY

                    # If hand to close to the upward pointing ranging sensor land
                    if is_close(multi_ranger.up):
                        keep_flying = False

                    # Perform the motion defined above
                    motion_commander.start_linear_motion(velocity_x, velocity_y, 0)

                    # For 0.1 seconds
                    time.sleep(0.1)

            # Done
            print('Demo terminated!')
