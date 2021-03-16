
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

# Plotting library
import matplotlib.pyplot as plt
import numpy as np


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


"""
    Data Acquisition class saving all the relevant data from the CrazyFlie
"""
class DataAcquisition:
 

    def __init__(self, link_uri):
        
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
        
        # Define a CrazyFlie object
        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

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
        
        if data['range.up'] < 400 :
            self.is_connected = False
            self._cf.close_link()
            
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

class DataVisualization:

    def __init__(self, fig):
        
        # Allocate an array that contains the hieght values
        self.range_down_array = [0] * 400
        self.time_array = [0] * 400
        
        self.fig = fig
        self.ax = fig.add_subplot(3,1,1) #
        self.ax.grid(b=True, color='k', linestyle='--')
        self.ax.set_ylabel('Laser Measurements')
        self.ax.set_xlabel('Time')
       
        self.bx = fig.add_subplot(3,1,2) #
        self.bx.grid(b=True, color='k', linestyle='--')
        self.bx.set_ylabel('Roll, Pitch, Yaw') 
        self.bx.set_xlabel('Time') 
        
        
        self.cx = fig.add_subplot(3,1,3) #
        self.cx.grid(b=True, color='k', linestyle='--')
        self.cx.set_ylabel('Left, Right, Front, Back')
        self.cx.set_xlabel('Time')
        
        self.line_agl, = self.ax.plot(0, 0,'b',label ="Downward Laser" )
        self.line_up, = self.ax.plot(0, 0,'r', label ="Upward Laser" )
        self.ax.legend(loc = 'upper left')
        
        self.line_roll, = self.bx.plot(0, 0,'b', label= "Roll")
        self.line_pitch, = self.bx.plot(0, 0,'r', label ="Pitch")
        self.line_yaw, = self.bx.plot(0, 0,'g', label = "Yaw") 
        self.bx.legend(loc='upper left')
        
        self.line_agl.set_linewidth(2)
        self.line_up.set_linewidth(2)
        
        self.line_left, = self.cx.plot(0, 0,'b', label="Left")
        self.line_right, = self.cx.plot(0, 0,'r', label="Right")
        self.line_front, = self.cx.plot(0, 0,'m', label ="Front")
        self.line_back, = self.cx.plot(0, 0,'c', label ="Back") 
        self.cx.legend(loc='upper left')
        
        self.line_left.set_linewidth(2)
        self.line_right.set_linewidth(2)
        self.line_front.set_linewidth(2)
        self.line_back.set_linewidth(2)
        
    def UpdateDataSubplot(self, t_data, data1, data2):
        self.line_agl.set_xdata(t_data)                		  		    
        self.line_agl.set_ydata(data1)
        
        self.line_up.set_xdata(t_data)
        self.line_up.set_ydata(data2)
        
        self.ax.relim()
        self.ax.autoscale_view(True,True,True)
        
        plt.draw()
        plt.pause(0.01)
        
    def UpdateSurroundSubplot(self, t_data, data1, data2, data3, data4):
        self.line_left.set_xdata(t_data)                		  		    
        self.line_left.set_ydata(data1)
        
        self.line_right.set_xdata(t_data)
        self.line_right.set_ydata(data2)
        
        self.line_front.set_xdata(t_data)                		  		    
        self.line_front.set_ydata(data3)
        
        self.line_back.set_xdata(t_data)
        self.line_back.set_ydata(data4)
        
        self.cx.relim()
        self.cx.autoscale_view(True,True,True)  
        
        plt.draw()
        plt.pause(0.01)

    def UpdateRPYSubplot(self, t_data, roll, pitch, yaw):
        self.line_roll.set_xdata(t_data)                		  		    
        self.line_roll.set_ydata(roll)
        
        self.line_pitch.set_xdata(t_data)
        self.line_pitch.set_ydata(pitch)
        
        self.line_yaw.set_xdata(t_data)                		  		    
        self.line_yaw.set_ydata(yaw)
        
        #print(pitch)
        self.bx.relim()
        self.bx.autoscale_view(True,True,True)
        
        plt.draw()
        plt.pause(0.01)           


URI = 'radio://0/80/2M/E7E7E7E7E6'

if __name__ == '__main__':
    
    
    # Set plot to animated
    plt.ion() 

    # Define a figure box of ertain size
    fig1 = plt.figure(figsize=(10.0, 11.5))
    fig1.suptitle('CrazyFlie Visualization', fontsize=16)
 
    # Start visualization
    dv = DataVisualization(fig1)
 
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Open defined CrazyFlie   
    da = DataAcquisition(URI)  #available[0][0])
 
    # Keep the application alive until we are disconnected.
    while da.is_connected:
        
        dv.UpdateDataSubplot(da.time_array,da.range_down_array, da.range_up_array)
        
        dv.UpdateRPYSubplot(da.time_array, da.roll_array, da.pitch_array, da.yaw_array)
        
        dv.UpdateSurroundSubplot(da.time_array, da.range_left_array, da.range_right_array, da.range_front_array, da.range_back_array)
        
        time.sleep(0.1)
        
    print('Done here')        
