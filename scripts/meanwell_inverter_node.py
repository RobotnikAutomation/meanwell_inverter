#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import rospy

import time, threading
from robotnik_msgs.msg import State, InverterStatus


import serial
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

DEFAULT_FREQ = 1.0
MAX_FREQ = 20.0

DEFAULT_MEANWELL_FREQ=1.0
MAX_READ_ERRORS = 10
MAX_READ_LENGTH_STRING = 128
MIN_READ_LENGTH_STRING = 5
TIME_PUBLISHING_FLAG = 20

class MeanwellInverterComponent:
    def __init__(self, args):

        self.node_name = rospy.get_name().replace('/','')
        self.desired_freq = args['desired_freq']
        # Checks value of freq
        if self.desired_freq <= 0.0:
            rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
            self.desired_freq = DEFAULT_FREQ
        if self.desired_freq > MAX_FREQ:
            rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, MAX_FREQ))
            self.desired_freq = MAX_FREQ
        self.real_freq = 0.0
        # Saves the state of the component
        self.state = None # State.INIT_STATE # is it better to use the switchToState because it is printed to log
        #        # Saves the previous state
        self.previous_state = None # State.INIT_STATE


        # flag to control the initialization of the component
        self.initialized = False
        # flag to control the initialization of ROS stuff
        self.ros_initialized = False
        # flag to control that the control loop is running
        self.running = False
        # Variable used to control the loop frequency
        self.time_sleep = 1.0 / self.desired_freq

        # Timer to publish state
        self.publish_state_timer = 1

        self.port = args['port']
        self.serial_device = None
        self.read_errors = 0
        self.command_stop = 'C100000000000000\r'
        self.command_start = 'C010000000000000\r'
        self.command_query = 'Q\r'
        self.query_response_size = 61 # message is 60 bytes + 1 of line return
        rospy.loginfo('%s:init: Serial port = %s', self.node_name, self.port)

        self.state_msg = State()
        self.inverter_status_msg = InverterStatus()

        self.switchToState(State.INIT_STATE)

    def setup(self):

        if self.initialized:
            rospy.logwarn("%s::setup: already initialized" % self.node_name)
            return 0

        rospy.loginfo("%s::setup" % self.node_name)

        try:
            self.serial_device = serial.Serial(
                port=self.port,
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=1,
                bytesize=8,
                xonxoff=False,
                dsrdtr=False,
                rtscts=False,
                timeout=0.1 #1/self.desired_freq
            )
        except serial.serialutil.SerialException, e:
            rospy.logerr('%s:setup: Error opening port %s'%(self.node_name, e))
            return -1

        self.initialized = True


    def shutdown(self):
        if self.running:
            rospy.logwarn("%st::shutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.initialized:
            rospy.logwarn("%s::shutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        rospy.loginfo("%s::shutdown",self.node_name)

        if self.serial_device != None and not self.serial_device.closed:
            self.serial_device.close()

        return 0


    def rosPublish(self):
        self.state_msg.state = self.state
        self.state_msg.state_description = self.stateToString(self.state)
        self.state_msg.desired_freq = self.desired_freq
        self.state_msg.real_freq = self.real_freq

        self.state_publisher.publish(self.state_msg)
        self.inverter_status_msg.header.stamp = rospy.Time.now()
        self.inverter_status_publisher.publish(self.inverter_status_msg)


    def rosSetup(self):

        if self.ros_initialized:
            rospy.logwarn("%s::rosSetup: already initialized" % self.node_name)
            return 0

        # Services
        self.toggle_srv = rospy.Service('~toggle', SetBool, self.handle_toggle_srv)
        # Publishers
        self.state_publisher = rospy.Publisher('~state', State, queue_size=10)
        self.inverter_status_publisher = rospy.Publisher('~inverter_status', InverterStatus, queue_size=10)

        self.ros_initialized = True
        return 0


    def rosShutdown(self):
        if self.running:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component is still running" % self.node_name)
            return -1
        elif not self.ros_initialized:
            rospy.logwarn("%s::rosShutdown: cannot shutdown because the component was not setup" % self.node_name)
            return -1

        self.state_publisher.unregister()

        self.ros_initialized = False


    def shutdownState(self):
        '''
            Actions performed in shutdown state
        '''
        if self.shutdown() == 0:
            self.switchToState(State.INIT_STATE)

        return


    def initState(self):
        '''
            Actions performed in init state
        '''

        if not self.initialized:
            self.setup()

        else:
            self.switchToState(State.STANDBY_STATE)


        return


    def standbyState(self):
        '''
            Actions performed in standby state
        '''
        self.switchToState(State.READY_STATE)

        return


    def emergencyState(self):
        '''
            Actions performed in emergency state
        '''

        return


    def failureState(self):
        '''
            Actions performed in failure state
        '''


        return


    def readyState(self):
        if self.writeToSerialDevice(self.command_query) <= 0:
            rospy.logerr("%s::readyState: error sending command for query" % self.node_name)
        ret, response = self.readLineFromSerialDevice()

        if ret == 0 and len(response) > 0:
            #print 'reading %d bytes' % len(response)
            #print response
            self.processQueryResponse(response)

        elif ret != 0:
            self.read_errors+=1

        if self.read_errors> MAX_READ_ERRORS:
            self.switchToState(State.FAILURE_STATE)

        return


    def processQueryResponse(self, response):
        """

        """
        # response is in the form "(v0, v1, v2, ...)\r", so we need to remove the parenthesis and the line return for processing
        if response[0] != '(': # if the first char is not a (, skip processing
            return

        values = response[1:-2].split()

        try:
            self.inverter_status_msg.ac_voltage = float(values[0])
            self.inverter_status_msg.dc_voltage = float(values[2])
            self.inverter_status_msg.load = float(values[1]) / 100.0
            self.inverter_status_msg.percentage = float(values[3]) / 100.0
            self.inverter_status_msg.temperature = float(values[4])
            self.inverter_status_msg.on = values[9][0] == '1'
        except (ValueError, TypeError):
            rospy.logerr("%s::processQueryResponse: error while processing query response (bad data in response)" % self.node_name)
            self.inverter_status_msg.ac_voltage = float('nan') 
            self.inverter_status_msg.dc_voltage = float('nan')
            self.inverter_status_msg.load = float('nan')
            self.inverter_status_msg.percentage = float('nan')
            self.inverter_status_msg.temperature = float('nan')
            # self.inverter_status_msg.on = False # as we cannot process the response, we can assume that it is on an error state, so it cannot be considered to be on

    def writeToSerialDevice(self, data):
        bytes_written = 0
        try:
            bytes_written = self.serial_device.write(data)
        except serial.serialutil.SerialException, e:
            rospy.logerr('%s:writeToSerialDevice: Error writing port %s'%(self.node_name, e))
            return -1
        except Exception, e:
            rospy.logerr('%s:writeToSerialDevice: Error writing port %s'%(self.node_name, e))
            return -1

        return bytes_written

    def readFromSerialDevice(self, size):
        data_read = ''
        try:
            data_read = self.serial_device.read(size)
        except serial.serialutil.SerialException, e:
            rospy.logerr('%s:readFromSerialDevice: Error reading port %s'%(self.node_name, e))
            return -1,''
        except Exception, e:
            rospy.logerr('%s:readFromSerialDevice: Error reading port %s'%(self.node_name, e))
            return -1,''

        return 0,data_read

    def readLineFromSerialDevice(self):
        ret = ''
        while True:
            try:
                c = self.serial_device.read(1)
                if c == '':
                    return 0,ret
                elif c == '\r' or c == '\n':
                    return 0,ret + c
                else:
                    ret += c
            except serial.serialutil.SerialException, e:
                rospy.logerr('%s:readFromSerialDevice: Error reading port %s'%(self.node_name, e))
                return -1,''
            except Exception, e:
                rospy.logerr('%s:readFromSerialDevice: Error reading port %s'%(self.node_name, e))
                return -1,''


    def handle_toggle_srv(self, request):
        self.switchToState(State.STANDBY_STATE) # switch to standby state until on/off operation has been completed

        everything_ok = False
        if request.data == True:
            # turn on
            if self.writeToSerialDevice(self.command_start) > 0:
                everything_ok = True
                rospy.logwarn("%s::handle_toggle_srv: turning on meanwell inverter" % self.node_name)
            else:
                everything_ok = False
                rospy.logerr("%s::handle_toggle_srv: error while sending command to turn on" % self.node_name)

        if request.data == False:
            # turn off
            if self.writeToSerialDevice(self.command_stop) > 0:
                everything_ok = True
                rospy.logwarn("%s::handle_toggle_srv: turning off meanwell inverter" % self.node_name)
            else:
                everything_ok = False
                rospy.logerr("%s::handle_toggle_srv: error while sending command to turn off" % self.node_name)

        # check if operation was succesfull
        response = SetBoolResponse()
        response.success = everything_ok
        return response


    def start(self):
        '''
            Runs ROS configuration and the main control loop
            @return: 0 if OK
        '''
        self.rosSetup()

        if self.running:
            return 0

        self.running = True

        self.controlLoop()

        return 0


    def controlLoop(self):
        '''
            Main loop of the component
            Manages actions by state
        '''

        while self.running and not rospy.is_shutdown():
            t1 = time.time()

            if self.state == State.INIT_STATE:
                self.initState()

            elif self.state == State.STANDBY_STATE:
                self.standbyState()

            elif self.state == State.READY_STATE:
                self.readyState()

            elif self.state == State.EMERGENCY_STATE:
                self.emergencyState()

            elif self.state == State.FAILURE_STATE:
                self.failureState()

            elif self.state == State.SHUTDOWN_STATE:
                self.shutdownState()

            self.allState()

            t2 = time.time()
            tdiff = (t2 - t1)


            t_sleep = self.time_sleep - tdiff

            if t_sleep > 0.0:
                try:
                    rospy.sleep(t_sleep)
                except rospy.exceptions.ROSInterruptException:
                    rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
                    self.running = False

            t3= time.time()
            self.real_freq = 1.0/(t3 - t1)

        self.running = False
        # Performs component shutdown
        self.shutdownState()
        # Performs ROS shutdown
        self.rosShutdown()
        rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)

        return 0


    def allState(self):
        '''
            Actions performed in all states
        '''
        self.rosPublish()

        return


    def switchToState(self, new_state):
        '''
            Performs the change of state
        '''
        if self.state != new_state:
            self.previous_state = self.state
            self.state = new_state
            rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))

        return


    def stateToString(self, state):
        '''
            @param state: state to set
            @type state: State
            @returns the equivalent string of the state
        '''
        if state == State.INIT_STATE:
            return 'INIT_STATE'

        elif state == State.STANDBY_STATE:
            return 'STANDBY_STATE'

        elif state == State.READY_STATE:
            return 'READY_STATE'

        elif state == State.EMERGENCY_STATE:
            return 'EMERGENCY_STATE'

        elif state == State.FAILURE_STATE:
            return 'FAILURE_STATE'

        elif state == State.SHUTDOWN_STATE:
            return 'SHUTDOWN_STATE'
        else:
            return 'UNKNOWN_STATE'


def main():
    rospy.init_node("meanwell_inverter")

    _name = rospy.get_name().replace('/','')

    arg_defaults = {
      'topic_state': 'state',
      'desired_freq': DEFAULT_MEANWELL_FREQ,
      'port' : '/dev/ttyUSB0', #'/dev/ttyUSB_MEANWELL',
    }

    args = {}

    for name in arg_defaults:
        try:
            if rospy.search_param(name):
                args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, _name))


    meanwell_inverter_node = MeanwellInverterComponent(args)

    meanwell_inverter_node.start()

if __name__ == "__main__":
    main()
