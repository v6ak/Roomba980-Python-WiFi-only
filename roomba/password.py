#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__version__ = "2.0a"
'''
Python 3.6
Quick Program to get blid and password from roomba

Nick Waterton 5th May 2017: V 1.0: Initial Release
Nick Waterton 22nd Dec 2020: V2.0: Updated for i and S Roomba versions, update to minimum python version 3.6
'''

from pprint import pformat
import json
import logging
import random
import socket
import ssl
import string
import sys
import time
from ast import literal_eval
import configparser

class Password(object):
    '''
    Get Roomba blid and password - only V2 firmware supported
    if IP is not supplied, class will attempt to discover the Roomba IP first.
    Results are written to a config file, default ".\config.ini"
    V 1.2.3 NW 9/10/2018 added support for Roomba i7
    V 1.2.5 NW 7/10/2019 changed PROTOCOL_TLSv1 to PROTOCOL_TLS to fix i7 software connection problem
    V 1.2.6 NW 12/11/2019 add cipher to ssl to avoid dh_key_too_small issue
    V 2.0 NW 22nd Dec 2020 updated for S and i versions plus braava jet m6, min version of python 3.6
    V 2.1 NW 9th Dec 2021 Added getting password from aws cloud.
    '''

    VERSION = __version__ = "2.1"
    
    config_dicts = ['data', 'mapsize', 'pmaps', 'regions']

    def __init__(self, address='255.255.255.255', file=".\config.ini", login=[]):
        self.address = address
        self.file = file
        self.login = None
        self.password = None
        if len(login) >= 2:
            self.login = login[0]
            self.password = login[1]
        self.log = logging.getLogger('Roomba.{}'.format(__class__.__name__))
        self.log.info("Using Password version {}".format(self.__version__))
        

    def receive_udp(self):
        #set up UDP socket to receive data from robot
        port = 5678
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(10)
        if self.address == '255.255.255.255':
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(("", port))  #bind all interfaces to port
        self.log.info("waiting on port: {} for data".format(port))
        message = 'irobotmcs'
        s.sendto(message.encode(), (self.address, port))
        roomba_dict = {}
        while True:
            try:
                udp_data, addr = s.recvfrom(1024)   #wait for udp data
                #self.log.debug('Received: Robot addr: {} Data: {}'.format(addr, udp_data))
                if udp_data and udp_data.decode() != message:
                    try:
                        #if self.address != addr[0]:
                        #    self.log.warning(
                        #        "supplied address {} does not match "
                        #        "discovered address {}, using discovered "
                        #        "address...".format(self.address, addr[0]))
                        
                        parsedMsg = json.loads(udp_data.decode())
                        if addr[0] not in roomba_dict.keys():
                            s.sendto(message.encode(), (self.address, port))
                            roomba_dict[addr[0]]=parsedMsg
                            self.log.info('Robot at IP: {} Data: {}'.format(addr[0], json.dumps(parsedMsg, indent=2)))
                    except Exception as e:
                        self.log.info("json decode error: {}".format(e))
                        self.log.info('RECEIVED: {}'.format(pformat(udp_data)))

            except socket.timeout:
                break
        s.close()
        return roomba_dict
        
    def get_password_from_roomba(self, addr):
        '''
        Send MQTT magic packet to addr
        this is 0xf0 (mqtt reserved) 0x05(data length) 0xefcc3b2900 (data)
        Should receive 37 bytes containing the password for roomba at addr
        This is is 0xf0 (mqtt RESERVED) length (0x23 = 35) 0xefcc3b2900 (magic packet), 
        followed by 0xXXXX... (30 bytes of password). so 7 bytes, followed by 30 bytes of password
        total of 37 bytes
        Uses 10 second timeout for socket connection
        '''
        data = b''
        packet = bytes.fromhex('f005efcc3b2900')
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)
        
        #context = ssl.SSLContext(ssl.PROTOCOL_TLS)
        context = ssl.SSLContext()
        #context.set_ciphers('DEFAULT@SECLEVEL=1:HIGH:!DH:!aNULL')
        wrappedSocket = context.wrap_socket(sock)
        
        try:
            wrappedSocket.connect((addr, 8883))
            self.log.debug('Connection Successful')
            try:
                wrappedSocket.send(packet)
                self.log.debug('Waiting for data')

                while len(data) < 37:
                    data_received = wrappedSocket.recv(1024)
                    data+= data_received
                    if len(data_received) == 0:
                        self.log.info("socket closed")
                        break
            finally:
                wrappedSocket.close()
        except Exception as e:
            data = b''
            if isinstance(e, socket.timeout):
                self.log.error('Connection Timeout Error (for {}): {}'.format(addr, e))
            elif isinstance(e, ConnectionRefusedError) or isinstance(e, OSError):
                if e.errno == 111:      #errno.ECONNREFUSED
                    self.log.error('Unable to Connect to roomba at ip {}, make sure nothing else is connected (app?), '
                                   'as only one connection at a time is allowed'.format(addr))
                elif e.errno == 113:    #errno.No Route to Host
                    self.log.error('Unable to contact roomba on ip {} is the ip correct?'.format(addr))
                else:
                    self.log.error("Connection Error (for {}): {}".format(addr, e))
            else:
                self.log.exception(e)

            self.log.error('Unable to get password from roomba')
            return None

        if len(data) <= 7:
            return None

        # Convert password to str
        password = data[7:]
        if b'\x00' in password:
            password = password[:password.find(b'\x00')]  # for i7 - has null termination
        return str(password.decode())


