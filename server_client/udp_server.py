#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 16:22:06 2021

@author: maartenuijtdehaag
"""

import socket
from datetime import datetime

class UDPServer:
    ''' A simple UDP Server '''

    def __init__(self, host, port):
        self.host = host    # Host address
        self.port = port    # Host port
        self.sock = None    # Socket

    def printwt(self, msg):
        ''' Print message with current date and time '''

        current_date_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print(f'[{current_date_time}] {msg}')

    def configure_server(self):
        ''' Configure the server '''

        # create UDP socket with IPv4 addressing
        self.printwt('Creating socket...')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.printwt('Socket created')

        # bind server to the address
        self.printwt(f'Binding server to {self.host}:{self.port}...')
        self.sock.bind((self.host, self.port))
        self.printwt(f'Server binded to {self.host}:{self.port}')

    def get_phone_no(self, name):
        ''' Get phone no for a given name '''

        phonebook = {'Alex': '1234567890', 'Bob': '1234512345'}

        if name in phonebook.keys():
            return f"{name}'s phone number is {phonebook[name]}"
        else:
            return f"No records found for {name}"

    def handle_request(self, data, client_address):
        ''' Handle the client '''

        # handle request
        name = data.decode('utf-8')
        resp = self.get_phone_no(name)
        self.printwt(f'[ REQUEST from {client_address} ]')
        print('\n', name, '\n')

        # send response to the client
        self.printwt(f'[ RESPONSE to {client_address} ]')
        self.sock.sendto(resp.encode('utf-8'), client_address)
        print('\n', resp, '\n')

    def wait_for_client(self):
        ''' Wait for a client '''

        try:
            # receive message from a client
            data, client_address = self.sock.recvfrom(1024)

            # handle client's request
            self.handle_request(data, client_address)

        except OSError as err:
            self.printwt(err)

    def shutdown_server(self):
        ''' Shutdown the UDP server '''

        self.printwt('Shutting down server...')
        self.sock.close()