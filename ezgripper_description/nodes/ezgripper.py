#!/usr/bin/python
#
#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##


# SAKE Robotics
#  
#  If you want to modify this program, you can find the "Main program" and 
#  "Main loop" by searching for these terms.  They exist near the end of this file.
#
#  If you want to modify the actions of the Joystick, search for "joy_callback"
#
#


import roslib;
import rospy
from sensor_msgs.msg import Joy

import serial
import time
import thread
import math
import sys

class ErrorResponse(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return "Dynamixel error: "+repr(self.value)

class CommunicationError(RuntimeError):
    def __init__(self, text):
        RuntimeError.__init__(self, text)



class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57600 ):
    #def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57142):
        self.dev_name = dev_name # stores it as a /dev-mapped string for Linux / Mac

        self.mutex = thread.allocate_lock()
        self.servo_dev = None

        self.acq_mutex()
        self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

    def send_serial(self, msg):
        # It is up to the caller to acquire / release mutex
        self.servo_dev.write( msg )

    def read_serial(self, nBytes=1):
        # It is up to the caller to acquire / release mutex
        rep = self.servo_dev.read( nBytes )
        return rep

    def _open_serial(self, baudrate):
        try:
            self.servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            # Closing the device first seems to prevent "Access Denied" errors on WinXP
            # (Conversations with Brian Wu @ MIT on 6/23/2010)
            self.servo_dev.close()  
            self.servo_dev.setParity('N')
            self.servo_dev.setStopbits(1)
            self.servo_dev.open()

            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            raise RuntimeError('lib_robotis: Serial port not found!\n')
        if(self.servo_dev == None):
            raise RuntimeError('lib_robotis: Serial port not found!\n')





class Robotis_Servo():
    ''' Class to use a robotis RX-28 or RX-64 servo.
    '''
    def __init__(self, USB2Dynamixel, servo_id ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
        '''

        # Error Checking
        if USB2Dynamixel == None:
            raise RuntimeError('lib_robotis: Robotis Servo requires USB2Dynamixel!\n')
        else:
            self.dyn = USB2Dynamixel

        # ID exists on bus?
        self.servo_id = servo_id
        try:
            self.read_address(3)
        except:
            print "Get ID failed once"
            self.dyn.servo_dev.flushOutput()
            self.dyn.servo_dev.flushInput()
            try:
                self.read_address(3)
            except:
                raise RuntimeError('lib_robotis: Error encountered.  Could not find ID (%d) on bus (%s), or USB2Dynamixel 3-way switch in wrong position.\n' %
                                   ( servo_id, self.dyn.dev_name ))

        # Set Return Delay time - Used to determine when next status can be requested
        data = self.read_address( 0x05, 1)
        self.return_delay = data[0] * 2e-6

    def flushAll(self):
        self.dyn.servo_dev.flushOutput()
        self.dyn.servo_dev.flushInput()

    def init_cont_turn(self):
        '''sets CCW angle limit to zero and allows continuous turning (good for wheels).
        After calling this method, simply use 'set_angvel' to command rotation.  This 
        rotation is proportional to torque according to Robotis documentation.
        '''
        self.write_address(0x08, [0,0])

    def kill_cont_turn(self):
        '''resets CCW angle limits to allow commands through 'move_angle' again
        '''
        self.write_address(0x08, [255, 3])

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        data = self.read_address( 0x2e, 1 )
        return data[0] != 0

    def read_voltage(self):
        ''' returns voltage (Volts)
        '''
        data = self.read_address( 0x2a, 1 )
        return data[0] / 10.

    def read_temperature(self):
        ''' returns the temperature (Celcius)
        '''
        data = self.read_address( 0x2b, 1 )
        return data[0]

    def read_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data = self.read_address( 0x28, 2 )
        load = data[0] + (data[1] >> 6) * 256
        if data[1] >> 2 & 1 == 0:
            return -1.0 * load
        else:
            return 1.0 * load

    def read_encoder(self):
        ''' returns position in encoder ticks
        '''
        data = self.read_address( 0x24, 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_word(self, addr):
        data = self.read_address( addr, 2 )
        value = data[0] + data[1] * 256
        return value

    def enable_torque(self):
        return self.write_address(0x18, [1])

    def disable_torque(self):
        return self.write_address(0x18, [0])

    def set_angvel(self, angvel):
        ''' angvel - in rad/sec
        '''     
        rpm = angvel / (2 * math.pi) * 60.0
        angvel_enc = int(round( rpm / 0.111 ))
        if angvel_enc<0:
            hi,lo = abs(angvel_enc) / 256 + 4, abs(angvel_enc) % 256
        else:
            hi,lo = angvel_enc / 256, angvel_enc % 256
        
        return self.write_address( 0x20, [lo,hi] )

    def write_id(self, id):
        ''' changes the servo id
        '''
        return self.write_address( 0x03, [id] )

    def write_baudrate(self, rate):
        return self.write_address( 0x04, [rate] )
        
    def write_word(self, addr, word):
        while word > 65535:
            word = word - 65536
        while word < 0:
            word = word + 65536
        hi,lo = word / 256, word % 256
        return self.write_address( addr, [lo,hi] )        
        
    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self.send_instruction( msg, self.servo_id )

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self.send_instruction( msg, self.servo_id )

    def send_instruction(self, instruction, id):
        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]
        
        self.dyn.acq_mutex()
        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except:
            self.dyn.rel_mutex()
            raise
        self.dyn.rel_mutex()
        
        if err != 0:
            self.process_err( err )

        return data

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def receive_reply(self):
        servo_data = self.dyn.read_serial( 1 )
        if servo_data != '\xff':
            raise RuntimeError('lib_robotis: Failed to receive start bytes\n')
        servo_data = self.dyn.read_serial( 1 )
        if servo_data == '\xff':
            servo_data = self.dyn.read_serial( 1 )

        servo_id =  servo_data
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received: %d\n' % ord(servo_id))
        data_len = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )
        data = self.dyn.read_serial( ord(data_len) - 2 )
        checksum = self.dyn.read_serial( 1 ) # I'm not going to check...
        return [ord(v) for v in data], ord(err)
        

    def send_serial(self, msg):
        """ sends the command to the servo
        """
        out = ''
        for m in msg:
            out += chr(m)
        self.dyn.send_serial( out )

    def check_overload_and_recover(self):
        cur_torque, e = self.read_wordX(34)
        if e & 32 != 0:
            print "Servo %d: status code %d, will try to recover"%(self.servo_id, e)
            self.write_wordX(71, 0)             # reset goal torque
            self.write_addressX(70, [0])        # torque control off
            self.write_wordX(34, TORQUE_LIMIT)  # restore torque limit
            cur_torque, e = self.read_wordX(34) # check stats
            if e & 32 == 0:
                print "Servo %d: recovery done, status code %d"%(self.servo_id, e)
            else:
                print "Servo %d: recovery failed, status code %d"%(self.servo_id, e)


def do_sake_calibration(self):
        self.write_address(6, [255,15,255,15] )   # 1) "Multi-Turn" - ON
        self.write_word(34, 500)                  # 2) "Torque Limit" to 500 (or so)
        self.write_address(24, [0])               # 3) "Torque Enable" to OFF
        
        self.write_address(70, [1])               # 1) Set "Goal Torque Mode" to ON
        self.write_word(71, 1024 + torque_hold)            # 2) Set "Goal Torque" Direction to CW and Value 50
        
        # wait until it stops
        time.sleep(2.0)
        
        
        position = self.read_word(36)
        multiturnoffset = self.read_word(20)                   # 4) Set "Multi turn offset" to 0
        self.write_address(70, [0])

        
        self.write_word(20, multiturnoffset-position)

        self.write_word(71, 1024 + 0)            # 3) Quickly turn "Goal Torque Mode" to OFF (to reduce load on motor)
        

def set_torque_mode(self, val):
    
        if val:
            print "torque on"
            self.write_address(70, [1])
        else:
            print "torque off"
            self.write_address(70, [0])

def joy_callback(joy):
    global grip_value, turn_value
    
    if joy.buttons[3] == 1: # Y
        set_torque_mode(grip_servo11, True)
#        set_torque_mode(grip_servo12, True)
#        set_torque_mode(grip_servo13, True)
        grip_servo11.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
#        grip_servo12.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
#        grip_servo13.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
    if joy.buttons[2] == 1: # X
        set_torque_mode(grip_servo11, False)
#        set_torque_mode(grip_servo12, False)
#        set_torque_mode(grip_servo13, False)
    if joy.buttons[1] == 1: # B
        set_torque_mode(grip_servo11, False)
#        set_torque_mode(grip_servo12, False)
#        set_torque_mode(grip_servo13, False)
        grip_servo11.write_word(30, grip_max)
#        grip_servo12.write_word(30, grip_max)
#        grip_servo13.write_word(30, grip_max)

    if joy.buttons[0] == 1: # A
        set_torque_mode(grip_servo11, True)
#        set_torque_mode(grip_servo12, True)
#        set_torque_mode(grip_servo13, True)
        grip_servo11.write_word(71, 1024 + torque_max)  # Set "Goal Torque" Direction to CW and Value
#        grip_servo12.write_word(71, 1024 + torque_max)  # Set "Goal Torque" Direction to CW and Value
#        grip_servo13.write_word(71, 1024 + torque_max)  # Set "Goal Torque" Direction to CW and Value
        time.sleep(0.5)
        grip_servo11.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
#        grip_servo12.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
#        grip_servo13.write_word(71, 1024 + torque_hold)  # Set "Goal Torque" Direction to CW and Value
    if joy.buttons[6] == 1: # return
        do_sake_calibration(grip_servo11)
#        do_sake_calibration(grip_servo12)
#        do_sake_calibration(grip_servo13)
        grip_servo11.write_word(30, grip_max)
#        grip_servo12.write_word(30, grip_max)
#        grip_servo13.write_word(30, grip_max)
    
    # gripper position
    if joy.buttons[13] == 1: # xpad driver mapping
    #if joy.axes[7] == 1.0: # xboxdrv mapping
        set_torque_mode(grip_servo11, False)
#        set_torque_mode(grip_servo12, False)
#        set_torque_mode(grip_servo13, False)
        grip_value = grip_value + grip_step
        if grip_value > grip_max:
            grip_value = grip_max
        print grip_value
        grip_servo11.write_word(30, grip_value)
#        grip_servo12.write_word(30, grip_value)
#        grip_servo13.write_word(30, grip_value)
    if joy.buttons[14] == 1:
    #if joy.axes[7] == -1.0:
        set_torque_mode(grip_servo11, False)
#        set_torque_mode(grip_servo12, False)
#        set_torque_mode(grip_servo13, False)
        grip_value = grip_value - grip_step
        if grip_value < grip_min:
            grip_value = grip_min
        print grip_value
        grip_servo11.write_word(30, grip_value)
#        grip_servo12.write_word(30, grip_value)
#        grip_servo13.write_word(30, grip_value)
            

# Main Programn

grip_value = 2800 #maximum open position for grippers
grip_max = 2800
grip_min = 0
grip_step = 45 # gripper step Cross Up and Cross Down

turn_value = 0
turn_max = 3072 # 270 degrees
turn_min = -3072
turn_step = 50

torque_max = 350 # maximum torque - MX-64=500, MX-106=350
torque_hold = 100 # holding torque - MX-64=100, MX-106=80

dyn = USB2Dynamixel_Device('/dev/ttyUSB0', 57142)

grip_servo11 = Robotis_Servo( dyn, 11 )
#grip_servo12 = Robotis_Servo( dyn, 12 )
#grip_servo13 = Robotis_Servo( dyn, 13 )

rospy.init_node('arm')
rospy.Subscriber("/joy", Joy, joy_callback)
rospy.loginfo("Armcontrol started")

r = rospy.Rate(20) # hz

do_sake_calibration(grip_servo11)        # Calibrate the gripper
grip_servo11.write_word(30, grip_max)    # Open the gripper to maximum after calibration
time.sleep(2.0)
#do_sake_calibration(grip_servo12)
#grip_servo12.write_word(30, grip_max)
#time.sleep(2.0)
#do_sake_calibration(grip_servo11)
#grip_servo13.write_word(30, grip_max)
#time.sleep(2.0)

# Main Loop

while not rospy.is_shutdown():

    try:
        grip_servo11.check_overload_and_recover()
       # grip_servo12.check_overload_and_recover()
       # grip_servo13.check_overload_and_recover()

    except CommunicationError, e:
        print "loop CommunicationError %s"%e
        grip_servo11.flushAll()
       # grip_servo12.flushAll()
       # grip_servo13.flushAll()
    except Exception, e:
        print "Exception: %s"%e
        grip_servo11.flushAll()
       # grip_servo12.flushAll()
       # grip_servo13.flushAll()

    r.sleep()

	

