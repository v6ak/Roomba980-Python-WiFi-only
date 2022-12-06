#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Python 3.6 Program to connect to Roomba vacuum cleaners, dcode json, and forward to mqtt
server

Nick Waterton 24th April 2017: V 1.0: Initial Release
Nick Waterton 4th July   2017  V 1.1.1: Fixed MQTT protocol version, and map
paths, fixed paho-mqtt tls changes
Nick Waterton 5th July   2017  V 1.1.2: Minor fixes, CV version 3 .2 support
Nick Waterton 7th July   2017  V1.2.0: Added -o option "roomOutline" allows
enabling/disabling of room outline drawing, added auto creation of css/html files
Nick Waterton 11th July  2017  V1.2.1: Quick (untested) fix for room outlines
if you don't have OpenCV
Nick Waterton 3rd Feb  2018  V1.2.2: Quick (untested) fix for running directly (ie not installed)
Nick Waterton 12th April 2018 V1.2.3: Fixed image rotation bug causing distorted maps if map rotation was not 0.
Nick Waterton 21st Dec 2018 V1.2.4: Fixed problem with findContours with OpenCV V4. Note V4.0.0-alpha still returns 3 values, and so won't work.
Nick Wateton 7th Oct 2019 V1.2.5: changed PROTOCOL_TLSv1 to PROTOCOL_TLS to fix i7 connection problem after F/W upgrade.
Nick Waterton 12th Nov 2019 V1.2.6: added set_ciphers('DEFAULT@SECLEVEL=1') to ssl context to work arounf dh_key_too_small error.
Nick Waterton 14th Jan 2020 V1.2.7: updated error code list.
Nick Waterton 16th march 2020 V 1.2.8 fixed __version__ for Pillow v7 (replaced with __version__)
Nick Waterton 24th April 2020 V 1.2.9 added possibility to send json commands to Roomba
Nick Waterton 24th dec 2020 V 2.0.0: Complete re-write
Nick Waterton 26th Februaury 2021 v 2.0.0b changed battery low and bin full handling, added 'no battery' (error 101).
Nick Waterton 3rd march 2021 v 2.0.0c changed battery low when docked, added callback handling and flags, added tank level,
                                      changed bin full handling, recovery from error condition mapping and added floormaps
                                      updated error list
Nick Waterton 27th March 2021 V2.0.0d Fixed floorplan offset on webpage in map.js.
Nick Waterton 28th March 2021 V2.0.0e Added invery x, y option
Nick Waterton 19th April 2021 V2.0.0f: added set_ciphers('DEFAULT@SECLEVEL=1') to ssl context to work arounf dh_key_too_small error (requred for ubuntu 20.04).
Nick Waterton 3rd May 2021 V2.0.0g: More python 3.8 fixes.
Nick Waterton 7th May 2021 V2.0.0h: added "ignore_run" after mission complete as still geting bogus run states
Nick Waterton 17th May 2021 V2.0.0i: mission state machine rework due to bogus states still being reported. increased max_distance to 500
'''

__version__ = "2.0.0i"

import asyncio
from ast import literal_eval
#from collections import OrderedDict, Mapping
from collections.abc import Mapping
from password import Password
import datetime
import json
import math
import logging
import os
import socket
import ssl
import sys
import time
import textwrap
import io
import configparser

# Import trickery
global HAVE_CV2
global HAVE_MQTT
global HAVE_PIL
HAVE_CV2 = False
HAVE_MQTT = False
HAVE_PIL = False
try:
    import paho.mqtt.client as mqtt
    HAVE_MQTT = True
except ImportError:
    print("paho mqtt client not found")
try:
    import cv2
    import numpy as np
    HAVE_CV2 = True
except ImportError:
    print("CV or numpy module not found, falling back to PIL")

try:
    from PIL import Image, ImageDraw, ImageFont, ImageFilter, ImageOps, ImageColor
    HAVE_PIL = True
except ImportError:
    print("PIL module not found, maps are disabled")
    
if sys.version_info < (3, 7):
    asyncio.get_running_loop = asyncio.get_event_loop
    
transparent = (0, 0, 0, 0)  #transparent colour

class Roomba(object):
    '''
    This is a Class for Roomba WiFi connected Vacuum cleaners and mops
    Requires firmware version 2.0 and above (not V1.0). Tested with Roomba 980, s9
    and braava M6.
    username (blid) and password are required, and can be found using the
    Password() class (in password.py - or can be auto discovered)
    Most of the underlying info was obtained from here:
    https://github.com/koalazak/dorita980 many thanks!
    The values received from the Roomba as stored in a dictionary called
    master_state, and can be accessed at any time, the contents are live, and
    will build with time after connection.
    This is not needed if the forward to mqtt option is used, as the events will
    be decoded and published on the designated mqtt client topic.
    '''

    VERSION = __version__ = "2.0i"

    states = {"charge"          : "Charging",
              "new"             : "New Mission",
              "run"             : "Running",
              "resume"          : "Running",
              "hmMidMsn"        : "Docking",
              "recharge"        : "Recharging",
              "stuck"           : "Stuck",
              "hmUsrDock"       : "User Docking",
              "completed"       : "Mission Completed",
              "cancelled"       : "Cancelled",
              "stop"            : "Stopped",
              "pause"           : "Paused",
              "evac"            : "Emptying",
              "hmPostMsn"       : "Docking - End Mission",
              "chargingerror"   : "Base Unplugged",
              ""                :  None}
    
    # from various sources
    _ErrorMessages = {
        0: "None",
        1: "Left wheel off floor",
        2: "Main brushes stuck",
        3: "Right wheel off floor",
        4: "Left wheel stuck",
        5: "Right wheel stuck",
        6: "Stuck near a cliff",
        7: "Left wheel error",
        8: "Bin error",
        9: "Bumper stuck",
        10: "Right wheel error",
        11: "Bin error",
        12: "Cliff sensor issue",
        13: "Both wheels off floor",
        14: "Bin missing",
        15: "Reboot required",
        16: "Bumped unexpectedly",
        17: "Path blocked",
        18: "Docking issue",
        19: "Undocking issue",
        20: "Docking issue",
        21: "Navigation problem",
        22: "Navigation problem",
        23: "Battery issue",
        24: "Navigation problem",
        25: "Reboot required",
        26: "Vacuum problem",
        27: "Vacuum problem",
        29: "Software update needed",
        30: "Vacuum problem",
        31: "Reboot required",
        32: "Smart map problem",
        33: "Path blocked",
        34: "Reboot required",
        35: "Unrecognised cleaning pad",
        36: "Bin full",
        37: "Tank needed refilling",
        38: "Vacuum problem",
        39: "Reboot required",
        40: "Navigation problem",
        41: "Timed out",
        42: "Localization problem",
        43: "Navigation problem",
        44: "Pump issue",
        45: "Lid open",
        46: "Low battery",
        47: "Reboot required",
        48: "Path blocked",
        52: "Pad required attention",
        53: "Software update required",
        65: "Hardware problem detected",
        66: "Low memory",
        68: "Hardware problem detected",
        73: "Pad type changed",
        74: "Max area reached",
        75: "Navigation problem",
        76: "Hardware problem detected",
        88: "Back-up refused",
        89: "Mission runtime too long",
        101: "Battery isn't connected",
        102: "Charging error",
        103: "Charging error",
        104: "No charge current",
        105: "Charging current too low",
        106: "Battery too warm",
        107: "Battery temperature incorrect",
        108: "Battery communication failure",
        109: "Battery error",
        110: "Battery cell imbalance",
        111: "Battery communication failure",
        112: "Invalid charging load",
        114: "Internal battery failure",
        115: "Cell failure during charging",
        116: "Charging error of Home Base",
        118: "Battery communication failure",
        119: "Charging timeout",
        120: "Battery not initialized",
        122: "Charging system error",
        123: "Battery not initialized",
    }

    def __init__(self, address=None, blid=None, password=None, topic="#",
                       roombaName="", file="./config.ini", log=None, webport=None):
        '''
        address is the IP address of the Roomba,
        leave topic as is, unless debugging (# = all messages).
        if a python standard logging object called 'Roomba' exists,
        it will be used for logging,
        or pass a logging object
        '''
        self.loop = asyncio.get_event_loop()
        self.debug = False
        if log:
            self.log = log
        else:
            self.log = logging.getLogger("Roomba.{}".format(roombaName if roombaName else __name__))
        if self.log.getEffectiveLevel() == logging.DEBUG:
            self.debug = True
        self.address = address
        # set the following to True to enable pretty printing of json data
        self.pretty_print = False
        self.roomba_port = 8883
        self.blid = blid
        self.password = password
        self.roombaName = roombaName
        self.file = file
        self.get_passwd = Password(file=file)
        self.topic = topic
        self.webport = webport
        self.ws = None
        self.args = None    #shadow class variable
        self.mqttc = None
        self.local_mqtt = False
        self.exclude = ""
        self.roomba_connected = False
        self.indent = 0
        self.master_indent = 0
        self.raw = False
        self.drawmap = False
        self.mapSize = None
        self.roomba_angle = 0
        self.old_x_y = None
        self.fnt = None
        self.home_pos = None
        self.angle = 0
        self.invert_x = self.invert_y = None    #mirror x,y
        self.current_state = None
        self.simulation = False
        self.simulation_reset = False
        self.max_distance = 500             #max distance to draw lines
        self.base = None                    #base map
        self.room_outline_contour = None
        self.room_outline = None
        self.floorplan = None
        self.floorplan_size = None
        self.previous_display_text = self.display_text = None
        self.master_state = {}
        self.update_seconds = 300           #update with all values every 5 minutes
        self.show_final_map = True
        self.client = None                  #Roomba MQTT client
        self.roombas_config = {}            #Roomba configuration loaded from config file
        self.history = {}
        self.timers = {}
        self.flags = {}
        self.max_sqft = None
        self.cb = None
        
        self.is_connected = asyncio.Event(loop=self.loop)
        self.q = asyncio.Queue()
        self.command_q = asyncio.Queue()            
        self.loop.create_task(self.process_q())
        self.loop.create_task(self.process_command_q())
        self.update = self.loop.create_task(self.periodic_update())

        if not all(field is not None for field in [self.address, self.blid, self.password]):
            if not self.configure_roomba():
                self.log.critical('Could not configure Roomba')
        else:
            self.roombas_config = {self.address: {
                                   "blid": self.blid,
                                   "password": self.password,
                                   "roomba_name": self.roombaName}}
                                   
        if self.webport:
            self.setup_webserver()
            
    async def event_wait(self, evt, timeout):
        '''
        Event.wait() with timeout
        '''
        try:
            await asyncio.wait_for(evt.wait(), timeout)
        except asyncio.TimeoutError:
            pass
        return evt.is_set()

    def setup_client(self):
        if self.client is None:
            if not HAVE_MQTT:
                print("Please install paho-mqtt 'pip install paho-mqtt' "
                      "to use this library")
                return False
            self.client = mqtt.Client(
                client_id=self.blid, clean_session=True,
                protocol=mqtt.MQTTv311)
            # Assign event callbacks
            self.client.on_message = self.on_message
            self.client.on_connect = self.on_connect
            self.client.on_publish = self.on_publish
            self.client.on_subscribe = self.on_subscribe
            self.client.on_disconnect = self.on_disconnect

            # Uncomment to enable debug messages
            #self.client.on_log = self.on_log

            self.log.info("Setting TLS")
            try:
                #self.client._ssl_context = None
                context = ssl.SSLContext()
                # Either of the following context settings works - choose one
                # Needed for 980 and earlier robots as their security level is 1.
                # context.set_ciphers('HIGH:!DH:!aNULL')
                context.set_ciphers('DEFAULT@SECLEVEL=1')
                self.client.tls_set_context(context)
            except Exception as e:
                self.log.exception("Error setting TLS: {}".format(e))

            # disables peer verification
            self.client.tls_insecure_set(True)
            self.client.username_pw_set(self.blid, self.password)
            self.log.info("Setting TLS - OK")
            return True
        return False

    def connect(self):
        '''
        just create async_connect task
        '''
        return self.loop.create_task(self.async_connect())

    async def async_connect(self):
        '''
        Connect to Roomba MQTT server
        '''
        if not all(field is not None for field in [self.address, self.blid, self.password]):
            self.log.critical("Invalid address, blid, or password! All these "
                              "must be specified!")
            return False
        count = 0
        max_retries = 3
        retry_timeout = 1
        while not self.roomba_connected:
            try:
                if self.client is None:
                    self.log.info("Connecting...")
                    self.setup_client()
                    await self.loop.run_in_executor(None, self.client.connect, self.address, self.roomba_port, 60)
                else:
                    self.log.info("Attempting to Reconnect...")
                    self.client.loop_stop()
                    await self.loop.run_in_executor(None, self.client.reconnect)
                self.client.loop_start()
                await self.event_wait(self.is_connected, 1)    #wait for MQTT on_connect to fire (timeout 1 second)
            except (ConnectionRefusedError, OSError) as e:
                if e.errno == 111:      #errno.ECONNREFUSED
                    self.log.error('Unable to Connect to roomba {}, make sure nothing else is connected (app?), '
                                   'as only one connection at a time is allowed'.format(self.roombaName))
                elif e.errno == 113:    #errno.No Route to Host
                    self.log.error('Unable to contact roomba {} on ip {}'.format(self.roombaName, self.address))
                else:
                    self.log.error("Connection Error: {} ".format(e))
                
                await asyncio.sleep(retry_timeout)
                self.log.error("Attempting retry Connection# {}".format(count))
                
                count += 1
                if count >= max_retries:
                    retry_timeout = 60
                    
            except asyncio.CancelledError:
                self.log.error('Connection Cancelled')
                break
            except Exception as e:
                #self.log.error("Error: {} ".format(e))
                self.log.exception(e)
                if count >= max_retries:
                    break
            
        if not self.roomba_connected:   
            self.log.error("Unable to connect to {}".format(self.roombaName))
        return self.roomba_connected

    def disconnect(self):
        try:
            self.loop.run_until_complete(self._disconnect())
        except RuntimeError:
            self.loop.create_task(self._disconnect())
    
    async def _disconnect(self):
        #if self.ws:
        #    await self.ws.cancel()
        tasks = [t for t in asyncio.Task.all_tasks() if t is not asyncio.Task.current_task()]
        [task.cancel() for task in tasks]
        self.log.info("Cancelling {} outstanding tasks".format(len(tasks)))
        await asyncio.gather(*tasks, return_exceptions=True)
        self.client.disconnect()
        if self.local_mqtt:
            self.mqttc.loop_stop()
        self.log.info('{} disconnected'.format(self.roombaName))
        
    def connected(self, state):
        self.roomba_connected = state
        self.publish('status', 'Online' if self.roomba_connected else 'Offline at {}'.format(time.ctime()))
        
    def on_connect(self, client, userdata, flags, rc):
        self.log.info("Roomba Connected")
        if rc == 0:
            self.connected(True)
            self.client.subscribe(self.topic)
            self.client.subscribe("$SYS/#")
        else:
            self.log.error("Connected with result code {}".format(str(rc)))
            self.log.error("Please make sure your blid and password are "
                           "correct for Roomba {}".format(self.roombaName))
            self.connected(False)
            self.client.disconnect()
        self.loop.call_soon_threadsafe(self.is_connected.set)

    def on_message(self, mosq, obj, msg):
        #print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        if self.exclude != "" and self.exclude in msg.topic:
            return
            
        if self.indent == 0:
            self.master_indent = max(self.master_indent, len(msg.topic))
            
        if not self.simulation:
            asyncio.run_coroutine_threadsafe(self.q.put(msg), self.loop)
            
    async def process_q(self):
        '''
        Main processing loop, run until program exit
        '''
        while True:
            try:
                if self.q.qsize() > 0:
                    self.log.warning('Pending event queue size is: {}'.format(self.q.qsize()))
                msg = await self.q.get()
                
                if not self.command_q.empty():
                    self.log.info('Command waiting in queue')
                    await asyncio.sleep(1)
                    
                log_string, json_data = self.decode_payload(msg.topic,msg.payload)
                self.dict_merge(self.master_state, json_data)

                if self.pretty_print:
                    self.log.info("%-{:d}s : %s".format(self.master_indent) % (msg.topic, log_string))
                else:
                    self.log.info("Received Roomba Data: {}, {}".format(str(msg.topic), str(msg.payload)))

                if self.raw:
                    self.publish(msg.topic, msg.payload)
                else:
                    await self.loop.run_in_executor(None, self.decode_topics, json_data)
                    
                self.q.task_done()
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.log.exception(e)

    async def periodic_update(self):
        '''
        publish status peridically
        '''
        while True:
            # default every 5 minutes
            await asyncio.sleep(self.update_seconds)
            if self.roomba_connected:
                self.log.info("Publishing master_state")
                await self.loop.run_in_executor(None, self.decode_topics, self.master_state)

    def on_publish(self, mosq, obj, mid):
        pass

    def on_subscribe(self, mosq, obj, mid, granted_qos):
        self.log.debug("Subscribed: {} {}".format(str(mid), str(granted_qos)))

    def on_disconnect(self, mosq, obj, rc):
        self.loop.call_soon_threadsafe(self.is_connected.clear)
        self.connected(False)
        if rc != 0:
            self.log.warning("Unexpected Disconnect! - reconnecting")
        else:
            self.log.info("Disconnected")

    def on_log(self, mosq, obj, level, string):
        self.log.info(string)

    def set_mqtt_client(self, mqttc=None, brokerFeedback='/roomba/feedback'):
        self.mqttc = mqttc
        if self.mqttc is not None:
            self.brokerFeedback = self.set_mqtt_topic(brokerFeedback)
    async def process_command_q(self):
        '''
        Command processing loop, run until program exit
        '''
        while True:
            value = await self.command_q.get()
            command = value.get('command')
            setting = value.get('setting')
            schedule = value.get('schedule')
            if command:
                await self.loop.run_in_executor(None, self._send_command, command)
            if setting:
                await self.loop.run_in_executor(None, self._set_preference, *setting)
            if schedule:
                await self.loop.run_in_executor(None, self._set_cleanSchedule, schedule)
            self.command_q.task_done()

    def publish(self, topic, message):
        if self.mqttc is not None and message is not None:
            topic = '{}/{}'.format(self.brokerFeedback, topic)
            self.log.debug("Publishing item: {}: {}".format(topic, message))
            self.mqttc.publish(topic, message)



    def dict_merge(self, dct, merge_dct):
        '''
        Recursive dict merge. Inspired by :meth:``dict.update()``, instead
        of updating only top-level keys, dict_merge recurses down into dicts
        nested to an arbitrary depth, updating keys. The ``merge_dct`` is
        merged into ``dct``.
        :param dct: dict onto which the merge is executed
        :param merge_dct: dct merged into dct
        :return: None
        '''
        for k, v in merge_dct.items():
            if (k in dct and isinstance(dct[k], dict)
                    and isinstance(merge_dct[k], Mapping)):
                self.dict_merge(dct[k], merge_dct[k])
            else:
                dct[k] = merge_dct[k]
                
    def recursive_lookup(self, search_dict, key, cap=False):
        '''
        recursive dictionary lookup
        if cap is true, return key if it's in the 'cap' dictionary,
        else return the actual key value
        '''
        for k, v in search_dict.items():
            if cap:
                if k == 'cap':
                    return self.recursive_lookup(v, key, False)
            elif k == key:
                return v 
            elif isinstance(v, dict) and k != 'cap':
                val = self.recursive_lookup(v, key, cap)
                if val is not None:
                    return val
        return None
        
    def is_setting(self, setting, search_dict=None):
        if search_dict is None:
            search_dict = self.master_state
        for k, v in search_dict.items():
            if k == setting:
                return True
            if isinstance(v, dict):
                if self.is_setting(setting, v):
                    return True
        return False

    def decode_payload(self, topic, payload):
        '''
        Format json for pretty printing, return string suitable for logging,
        and a dict of the json data
        '''
        indent = self.master_indent + 31 #number of spaces to indent json data

        try:
            # if it's json data, decode it (use OrderedDict to preserve keys
            # order), else return as is...
            json_data = json.loads(
                payload.decode("utf-8").replace(":nan", ":NaN").\
                replace(":inf", ":Infinity").replace(":-inf", ":-Infinity"))  #removed object_pairs_hook=OrderedDict
            # if it's not a dictionary, probably just a number
            if not isinstance(json_data, dict):
                return json_data, dict(json_data)
            json_data_string = "\n".join((indent * " ") + i for i in \
                (json.dumps(json_data, indent = 2)).splitlines())

            formatted_data = "Decoded JSON: \n%s" % (json_data_string)

        except ValueError:
            formatted_data = payload

        if self.raw:
            formatted_data = payload

        return formatted_data, dict(json_data)

    def decode_topics(self, state, prefix=None):
        '''
        decode json data dict, and publish as individual topics to
        brokerFeedback/topic the keys are concatenated with _ to make one unique
        topic name strings are expressly converted to strings to avoid unicode
        representations
        '''
        for k, v in state.items():
            if isinstance(v, dict):
                if prefix is None:
                    self.decode_topics(v, k)
                else:
                    self.decode_topics(v, prefix+"_"+k)
            else:
                if isinstance(v, list):
                    newlist = []
                    for i in v:
                        if isinstance(i, dict):
                            for ki, vi in i.items():
                                newlist.append((str(ki), vi))
                        else:
                            if not isinstance(i, str):
                                i = str(i)
                            newlist.append(i)
                    v = newlist
                if prefix is not None:
                    k = prefix+"_"+k
                # all data starts with this, so it's redundant
                k = k.replace("state_reported_","")
                    
                self.publish(k, str(v))

        if prefix is None:
            self.update_state_machine()
            
    async def get_settings(self, items):
        result = {}
        if not isinstance(items, list):
            items = [items]
        for item in items:
            value = await self.loop.run_in_executor(None, self.get_property, item)
            result[item] = value
        return result
        
    def get_error_message(self, error_num):
        try:
            error_message = self._ErrorMessages[error_num]
        except KeyError as e:
            self.log.warning(
                "Error looking up error message {}".format(e))
            error_message = "Unknown Error number: {}".format(error_num)
        return error_message
        
    def publish_error_message(self):
        self.publish("error_message", self.error_message)
            
    def get_property(self, property, cap=False):
        '''
        Only works correctly if property is a unique key
        '''
        if property in ['cleanSchedule', 'langs']:
            value = self.recursive_lookup(self.master_state, property+'2', cap)
            if value is not None:
                return value
        return self.recursive_lookup(self.master_state, property, cap)
        
    @property    
    def co_ords(self):
        co_ords = self.pose
        if isinstance(co_ords, dict):
            return {'x': -co_ords['point']['y'] if self.invert_x else co_ords['point']['y'],
                    'y': -co_ords['point']['x'] if self.invert_y else co_ords['point']['x'],
                    'theta':co_ords['theta']}
        return self.zero_coords()
        
    @property
    def error_num(self):
        try:
            return self.cleanMissionStatus.get('error')
        except AttributeError:
            pass
        return 0
        
    @property
    def error_message(self):
        return self.get_error_message(self.error_num)
        
    def update_precent_complete(self):
        try:
            sq_ft = self.get_property("sqft")
            if self.max_sqft and sq_ft is not None:
                percent_complete = int(sq_ft)*100//self.max_sqft
                self.publish("roomba_percent_complete", percent_complete)
                return percent_complete
        except (KeyError, TypeError):
            pass
        return None
        
    def update_history(self, property, value=None, cap=False):
        '''
        keep previous value
        '''
        if value is not None:
            current = value
        else:
            current = self.get_property(property, cap)
        if isinstance(current, dict):
            current = current.copy()
        previous = self.history.get(property, {}).get('current')
        if previous is None:
            previous = current
        self.history[property] = {'current' : current,
                                  'previous': previous}
        return current
        
    
    def is_set(self, name):
        return self.timers.get(name, {}).get('value', False)
        
    def update_state_machine(self, new_state = None):
        '''
        Roomba progresses through states (phases), current identified states
        are:
        ""              : program started up, no state yet
        "run"           : running on a Cleaning Mission
        "hmUsrDock"     : returning to Dock
        "hmMidMsn"      : need to recharge
        "hmPostMsn"     : mission completed
        "charge"        : charging
        "stuck"         : Roomba is stuck
        "stop"          : Stopped
        "pause"         : paused
        "evac"          : emptying bin
        "chargingerror" : charging base is unplugged

        available states:
        states = {"charge"          : "Charging",
                  "new"             : "New Mission",
                  "run"             : "Running",
                  "resume"          : "Running",
                  "hmMidMsn"        : "Docking",
                  "recharge"        : "Recharging",
                  "stuck"           : "Stuck",
                  "hmUsrDock"       : "User Docking",
                  "completed"       : "Mission Completed",
                  "cancelled"       : "Cancelled",
                  "stop"            : "Stopped",
                  "pause"           : "Paused",
                  "evac"            : "Emptying",
                  "hmPostMsn"       : "Docking - End Mission",
                  "chargingerror"   : "Base Unplugged",
                  ""                :  None}

        Normal Sequence is "" -> charge -> run -> hmPostMsn -> charge
        Mid mission recharge is "" -> charge -> run -> hmMidMsn -> charge
                                   -> run -> hmPostMsn -> charge
        Stuck is "" -> charge -> run -> hmPostMsn -> stuck
                    -> run/charge/stop/hmUsrDock -> charge
        Start program during run is "" -> run -> hmPostMsn -> charge
        Note: Braava M6 goes run -> hmPostMsn -> run -> charge when docking
        Note: S9+ goes run -> hmPostMsn -> charge -> run -> charge on a training mission (ie cleanMissionStatus_cycle = 'train')
        Note: The first 3 "pose" (x, y) co-ordinate in the first 10 seconds during undocking at mission start seem to be wrong
              for example, during undocking:
              {"x": 0, "y": 0},
              {"x": -49, "y": 0},
              {"x": -47, "y": 0},
              {"x": -75, "y": -11}... then suddenly becomes normal co-ordinates
              {"x": -22, "y": 131}
              {"x": -91, "y": 211}
              also during "hmPostMsn","hmMidMsn", "hmUsrDock" the co-ordinates system also seems to change to bogus values
              For example, in "run" phase, co-ordinates are reported as:
              {"x": -324, "y": 824},
              {"x": -324, "y": 826} ... etc, then some time after hmMidMsn (for example) they change to:
              {"x": 417, "y": -787}, which continues for a while
              {"x": 498, "y": -679}, and then suddenly changes back to normal co-ordinates
              {"x": -348, "y": 787},
              {"x": -161, "y": 181},
              {"x": 0, "y": 0}
              
              For now use self.distance_betwwen() to ignore large changes in position

        Need to identify a new mission to initialize map, and end of mission to
        finalise map.
        mission goes from 'none' to 'clean' (or another mission name) at start of mission (init map)
        mission goes from 'clean' (or other mission) to 'none' at end of missions (finalize map)
        Anything else = continue with existing map
        '''
        if new_state is not None:
            self.current_state = self.states[new_state]
            self.log.info("set current state to: {}".format(self.current_state))
            self.draw_map(True)
            return    
            
        self.publish_error_message()                #publish error messages
        self.update_precent_complete()
        mission = self.update_history("cycle")      #mission
        phase = self.update_history("phase")        #mission phase
        self.update_history("pose")                 #update co-ordinates
        
        if self.cb is not None:                     #call callback if set
            self.cb(self.master_state)
        
        if phase is None or mission is None:
            return
        
        current_mission = self.current_state
        
        if self.debug:
            self.timer('ignore_coordinates')
            current_mission = None  #force update of map

        self.log.info('current_state: {}, current phase: {}, mission: {}, mission_min: {}, recharge_min: {}, co-ords changed: {}'.format(self.current_state,
                                                                                                                    phase,
                                                                                                                    mission,
                                                                                                                    self.mssnM,
                                                                                                                    self.rechrgM,
                                                                                                                    self.changed('pose')))

        if phase == "charge":
            #self.set_history('pose', self.zero_pose())
            current_mission = None
            
        if self.current_state == self.states["new"] and phase != 'run':
            self.log.info('waiting for run state for New Missions')
            if time.time() - self.timers['start'] >= 20:
                self.log.warning('Timeout waiting for run state')
                self.current_state = self.states[phase]

        elif phase == "run" and (self.is_set('ignore_run') or mission == 'none'):
            self.log.info('Ignoring bogus run state')
            
        elif phase == "charge" and mission == 'none' and self.is_set('ignore_run'):
            self.log.info('Ignoring bogus charge/mission state')
            self.update_history("cycle", self.previous('cycle'))
            
        elif phase in ["hmPostMsn","hmMidMsn", "hmUsrDock"]:
            self.timer('ignore_run', True, 10)
            self.current_state = self.states[phase]
            
        elif self.changed('cycle'): #if mission has changed
            if mission != 'none':
                self.current_state = self.states["new"]
                self.timers['start'] = time.time()
                if isinstance(self.sku, str) and self.sku[0].lower() in ['i', 's', 'm']:
                    #self.timer('ignore_coordinates', True, 30)  #ignore updates for 30 seconds at start of new mission
                    pass
            else:
                self.timers.pop('start', None)
                if self.bin_full:
                    self.current_state = self.states["cancelled"]
                else:
                    self.current_state = self.states["completed"]
                self.timer('ignore_run', True, 5)  #still get bogus 'run' states after mission complete.
            
        elif phase == "charge" and self.rechrgM:
            if self.bin_full:
                self.current_state = self.states["pause"]
            else:
                self.current_state = self.states["recharge"]
            
        else:
            try:
                self.current_state = self.states[phase]
            except KeyError:
                self.log.warning('phase: {} not found in self.states'.format(phase))

        if self.current_state != current_mission:
            self.log.info("updated state to: {}".format(self.current_state))

        self.publish("state", self.current_state)
        
        if self.is_set('ignore_coordinates') and self.current_state != self.states["new"]:
            self.log.info('Ignoring co-ordinate updates')
        else:
            self.draw_map(current_mission != self.current_state)
            



















if __name__ == '__main__':
    from roomba_direct import main
    main()
