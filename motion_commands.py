
"""
    Avionics Systems for UAS

    Example that shows how you can define the motion of the CrazyFlie through commands

    Copyright (c) 2020 - FF-ILR-TUBerlin
"""

# Packages (libraries) required
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Unique radio link identifier between computer and Craziflie UAS
# URI = 'radio://0/80/2M'
URI = 'radio://0/80/2M/E7E7E7E7E9'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Main program loop
if __name__ == '__main__':
    
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Create a synchronous connection with the UAS, UAS is now known as 'scf'
    with SyncCrazyflie(URI) as scf:
        
        # Create a motion commandor (tools that let's you change the motion of the UAS) for 'scf'
        with MotionCommander(scf) as mc:
            
            # -------------------------------------
            # Motion #1: Take off
            # -------------------------------------
            print('Taking off!')
            time.sleep(0.1)
            
            print('Moving up 1.0m at 0.2m/s')
            mc.up(0.5,0.2)
            
            # Wait a bit (i.e. 5 seconds) before doing the next step
            time.sleep(5) 
            
            # -------------------------------------
            # Motion #2: Moving forward
            # -------------------------------------
            print('Moving forward 1.5m at 0.5m/s')
            mc.forward(0.3,0.5)
            
            # Wait a bit before doing the next step
            time.sleep(5)
            
            # # -------------------------------------
            # # Motion #3: flying an arc
            # # -------------------------------------
            # print('Doing a 180deg circle to the left');
            # mc.circle_left(0.5, velocity=0.5, angle_degrees=180) 
            
            # # Wait a bit before doing the next step
            # time.sleep(5)

            # # -------------------------------------
            # # Motion #4: moving downward
            # # -------------------------------------
            # print('Moving down 0.5m')
            # mc.down(0.5)
            
            # # Wait a bit before doing the next step
            # time.sleep(5)


            # # -------------------------------------
            # # Motion #5: flying forward 
            # # -------------------------------------
            # print('Moving forward 1m at 0.5m/s')
            # mc.forward(1.0,0.5)

            # -------------------------------------
            # Motion #6: landing -  
            # -------------------------------------
            # We land when the MotionCommander goes out of scope (outside the 'with' loop)
            print('Landing!')
            print('Demo terminated') 
            