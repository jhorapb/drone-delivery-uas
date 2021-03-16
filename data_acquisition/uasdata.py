
"""
    Avionics Systems for UAS

    Example that obtains various data fields from the CrazyFlie
    and shows them on the screen

    In addition it computes the range from the range sesnors,
    turns them input a point cloud and plots them

    Copyright (c) 2020 - FF-ILR-TUBerlin
"""


import logging
import time


# CrazyFlie library
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

import numpy as np


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


"""
    Data Acquisition class saving all the relevant data from the CrazyFlie
"""
class DataAcquisition:

    FRONT = 'range.front'
    BACK = 'range.back'
    LEFT = 'range.left'
    RIGHT = 'range.right'
    UP = 'range.up'
    DOWN = 'range.zrange'

    def __init__(self, link_uri='', rate_ms=100, zranger=False, crazyflie=None):
        
        N = 400
        
        # Allocate an array that contains the hieght values
        self.range_down_array = [np.nan] * N
        self.range_up_array = [np.nan] * N
        self.range_left_array = [np.nan] * N
        self.range_right_array = [np.nan] * N
        self.range_front_array = [np.nan] * N
        self.range_back_array = [np.nan] * N
        
        self.roll_array = [np.nan] * N
        self.pitch_array = [np.nan] * N
        self.yaw_array = [np.nan] * N
        
        self.time_array = [np.nan] * N
        
        if isinstance(crazyflie, SyncCrazyflie):
            self._cf = crazyflie.cf
        else:
            # self._cf = crazyflie
            # Define a CrazyFlie object
            print('here')
            self._cf = Crazyflie(rw_cache='./cache')
            self._cf.open_link(link_uri)

        self._log_config = self._create_log_config(rate_ms)
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie

    def start(self):
        self._cf.log.add_config(self._log_config)
        self._log_config.start()

    def stop(self):
        self._log_config.delete()

    def _create_log_config(self, rate_ms):
        log_config = LogConfig('multiranger', rate_ms)
        log_config.add_variable(self.FRONT)
        log_config.add_variable(self.BACK)
        log_config.add_variable(self.LEFT)
        log_config.add_variable(self.RIGHT)
        log_config.add_variable(self.UP)
        log_config.add_variable(self.DOWN)

        log_config.data_received_cb.add_callback(self._data_received)

        return log_config

    def _connected(self, link_uri):
        
        print('Connected to %s' % link_uri)
        
        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Meas', period_in_ms=100)
        self._lg_stab.add_variable('stabilizer.roll')
        self._lg_stab.add_variable('stabilizer.pitch')
        self._lg_stab.add_variable('stabilizer.yaw')
        self._lg_stab.add_variable('range.front')
        self._lg_stab.add_variable('range.back')
        self._lg_stab.add_variable('range.left')
        self._lg_stab.add_variable('range.right')
        self._lg_stab.add_variable('range.up')
        self._lg_stab.add_variable('range.zrange')
        
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            
            # Start the logging
            self._lg_stab.start()
            
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
    
    # Callback from the log API when an error occurs
    def _stab_log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    # Callback from a the log API when data arrives
    def _stab_log_data(self, timestamp, data, logconf):
        
        # Check if we need to stop
        if data['range.up'] < 400 :
            self.is_connected = False
            self._cf.close_link()
            
        print('Distance to ground: %f at time %d' %  (data['range.zrange'],timestamp))
            
        # Add the data to the arrays
        
        self.roll_array.append(data['stabilizer.roll'])
        del self.roll_array[0]
        
        self.pitch_array.append(data['stabilizer.pitch'])
        del self.pitch_array[0]
        
        self.yaw_array.append(data['stabilizer.yaw'])
        del self.yaw_array[0]
        
        self.range_left_array.append(data['range.left'])
        del self.range_left_array[0]
        
        self.range_right_array.append(data['range.right'])
        del self.range_right_array[0]
        
        self.range_front_array.append(data['range.front'])
        del self.range_front_array[0]
        
        self.range_back_array.append(data['range.back'])
        del self.range_back_array[0]
        
        self.range_down_array.append(data['range.zrange'])
        del self.range_down_array[0]
        
        self.range_up_array.append(data['range.up'])
        del self.range_up_array[0]
        
        self.time_array.append(timestamp)
        del self.time_array[0]  
  
    # Callback when connection initial connection fails 
    # (i.e no Crazyflie at the specified address)
    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False
        
    # Callback when disconnected after a connection has been made 
    # (i.e Crazyflie moves out of range)
    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    # Callback when the Crazyflie is disconnected (called in all cases)
    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False



URI = 'radio://0/80/2M/E7E7E7E7E6'

if __name__ == '__main__':
    
 
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Open defined CrazyFlie   
    da = DataAcquisition(URI)  #available[0][0])
 
    # Keep the application alive until we are disconnected.
    while da.is_connected:
        
        time.sleep(0.1)
        
    print('Done here')        
