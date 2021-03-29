#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 16:43:12 2021

@author: maartenuijtdehaag
"""
import socket
import time
from datetime import datetime

class UDPClient:
    ''' A simple UDP Client '''

    def __init__(self, host, port):
        self.host = host    # Host address
        self.port = port    # Host port
        self.sock = None    # Socket

    def printwt(self, msg):
        ''' Print message with current date and time '''

        current_date_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print(f'[{current_date_time}] {msg}')

    def configure_client(self):
        ''' Configure the client to use UDP protocol with IPv4 addressing '''

        # create UDP socket with IPv4 addressing
        self.printwt('Creating UDP/IPv4 socket ...')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.printwt('Socket created')

    def interact_with_server(self, data_in):
        ''' Send request to a UDP Server and receive reply from it. '''

        try:

            # send data to server
            self.printwt('Sending position information to U-space server...')
            
            self.sock.sendto(data_in.encode('utf-8'), (self.host, self.port))
            self.printwt('[ SENT ]')
            self.printwt(data_in)
           
            # receive data from server
            resp, server_address = self.sock.recvfrom(1024)
            self.printwt('[ RECEIVED ]')
            self.printwt(resp.decode())
            
            print('\n')
            time.sleep(1)
            
            return resp.decode()
            

        except OSError as err:
            print(err)

    def close_client(self):
            # close socket
            self.printwt('Closing socket...')
            self.sock.close()
            self.printwt('Socket closed')
            
def check_for_conflict(database, resp, uri, sector):
    ''' can example: can store all data in a local database copy
        and check if any of the other drones is in the same space '''
    temp = resp.split(',')
    count = 1
    while count < len(temp)  :
        database[temp[count]] = temp[count+1] + ',' + temp[count+2] 
        
        if (temp[count+1] == sector) and (temp[count] != uri):
            print('Conflict found with UAS '  + temp[count] + '\n')
      
        count = count + 3
        
def main():
    ''' Create a UDP Client, send message to a UDP Server and receive reply. '''

    database = {
                'E7E7E7E7E1': ',',
                'E7E7E7E7E2': ',',
                'E7E7E7E7E3': ',',
                'E7E7E7E7E4': ',',
                'E7E7E7E7E5': ',',
                'E7E7E7E7E6': ',',
                'E7E7E7E7E7': ',', 
                'E7E7E7E7E8': ',',
                'E7E7E7E7E9': ','
                }

    #udp_client = UDPClient('192.168.1.210', 50020)
    udp_client = UDPClient('127.0.0.1', 4444)
    udp_client.configure_client()
    
    ''' examples '''
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E5,AB,0.3')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'AB')
    time.sleep(1)
    
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E5,AB,0.5')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'AB')
    time.sleep(1)
    
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E5,BC1,0.5')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'BC1')
    time.sleep(1)
    
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E5,BC1,0.3')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'BC1')
    time.sleep(1)
    
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E5,BC2,0.3')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'BC2')
    time.sleep(1)
    
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E6,BC2,0.3')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'BC2')
    time.sleep(1)
    
    resp = udp_client.interact_with_server('$POS,E7E7E7E7E5,C,0.3')
    check_for_conflict(database, resp, 'E7E7E7E7E5', 'C')

    
    udp_client.close_client()

if __name__ == '__main__':
    main()
