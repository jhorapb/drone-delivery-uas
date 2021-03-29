#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 16:18:41 2021

@author: maartenuijtdehaag
"""
import socket
import threading
import time
import udp_server
from datetime import datetime

class UDPServerMultiClient(udp_server.UDPServer):
    ''' A simple UDP Server for handling multiple clients '''

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
    
    def __init__(self, host, port):
        super().__init__(host, port)
        self.socket_lock = threading.Lock()

    def handle_request(self, data, client_address):
        ''' Handle the client '''

        # handle request
        print('\n')
        data_received = data.decode('utf-8')
        self.printwt(f'[ REQUEST from {client_address} ]')
        self.printwt(data_received)
        chunks = data_received.split(',')
        self.database[chunks[1]] = chunks[2] + ',' + chunks[3]
        resp = '$POSALL'
        for ii in self.database:
            resp = resp + ',' + ii + ',' + self.database[ii]
            
        # send response to the client
        self.printwt(f'[ RESPONSE to {client_address} ]')
        with self.socket_lock:
            self.sock.sendto(resp.encode('utf-8'), client_address)
        self.printwt(resp)

    def wait_for_client(self):
        ''' Wait for clients and handle their requests '''

        try:
            while True: # keep alive

                try: # receive request from client
                    data, client_address = self.sock.recvfrom(1024)

                    c_thread = threading.Thread(target = self.handle_request,
                                            args = (data, client_address))
                    c_thread.daemon = True
                    c_thread.start()

                except OSError as err:
                    self.printwt(err)

        except KeyboardInterrupt:
            self.shutdown_server()

def main():
    ''' Create a UDP Server and handle multiple clients simultaneously '''

    udp_server_multi_client = UDPServerMultiClient('127.0.0.1', 4444)
    #udp_server_multi_client = UDPServerMultiClient('192.168.1.210', 50020)
    udp_server_multi_client.configure_server()
    udp_server_multi_client.wait_for_client()

if __name__ == '__main__':
    main()