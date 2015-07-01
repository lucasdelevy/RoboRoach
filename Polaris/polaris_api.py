#!/usr/bin/env python

"""
Polaris Spectra API for Ubuntu 12.04 LTS, tested only on revision G001.005

# Copyright (c) 2013
# Automation and Robotics Lab (LARA) at University of Brasilia (UnB)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Automation and Robotics Lab (LARA) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
"""

##############################################################################
#                                IMPORTS
##############################################################################
import serial
import struct

import sys
from select import select as sel

##############################################################################
#                                AUTHORSHIP
##############################################################################

__author__  = "Murilo M. Marinho"
__email__   = "murilomarinho@lara.unb.br"
__license__ = "LGPL"

##############################################################################
#                            MAIN ROUTINE (FOR TESTING)
##############################################################################

def main():

  # Initialize PolarisDriver object
  polaris_driver = PolarisDriver(port='/dev/ttyUSB0')  

  print 'Opening serial port...'
  polaris_driver.open()
  print 'Serial port OK.'

  print polaris_driver._apirev()

  polaris_driver.initStrayMarkerTracker()

  while True:
      rlist, _, _, = sel([sys.stdin],[],[],0.01)    
      if rlist:
        break

      print polaris_driver.getPositionFromBX("1801")

  print '\nStop tracking mode...'
  polaris_driver.stopTracking()
  print 'Tracking mode started successfully.'    

  print 'Closing serial port...'
  polaris_driver.close()
  print 'End.'


##############################################################################
#                            CLASS PortHandler
##############################################################################

class PortHandler:

  def __init__(self, string = None):

    # Creating object with no arguments
    if(string is None):
      self.numhandles = 0
      self.handles = []
      return

    self.numhandles = int(string[0:2])
    self.handles = []
    
    # Iterate and add handles
    for i in range(self.numhandles):
      port_id     = string[i*5+2 : i*5+4]
      port_status = int(string[i*5+4 : i*5+6])
      handle = PortHandle(port_id,port_status);
      self.handles.append(handle)

  def __str__(self):
    print 'Number of Handles: ',self.numhandles
    for handle in self.handles:
      print handle
    return ''


class PortHandle:
  def __init__(self, id, status):
    self.id = id
    self.status = status
    self.tool   = Tool()

  def __str__(self):
    return 'Handle '+str(self.id)+' State: '+str(self.status)


class Tool:

  TOOL_STATUS_VALID    = '01'
  TOOL_STATUS_MISSING  = '02'
  TOOL_STATUS_DISABLED = '04'

  def __init__(self):
    self.trans  = Translation()
    self.rot    = Quaternion()
    self.status = self.TOOL_STATUS_DISABLED
    self.error  = 0.0
    self.frame_number = 0

  def updateFrameNumber(self,frame_number):
    self.frame_number = frame_number

  def updateError(self,error):
    self.error = error

  def updateRot(self,q0,q1,q2,q3):
    self.rot.q0=q0
    self.rot.q1=q1
    self.rot.q2=q2
    self.rot.q3=q3

  def updateTrans(self,x,y,z):
    self.trans.x=x
    self.trans.y=y
    self.trans.z=z

class Translation:
  def __init__(self,x=0.0,y=0.0,z=0.0):
    self.x = x
    self.y = y
    self.z = z


class Quaternion:
  def __init__(self,q0=0.0,q1=0.0,q2=0.0,q3=0.0):
    self.q0=q0
    self.q1=q1
    self.q2=q2
    self.q3=q3

##############################################################################
#                            CLASS PolarisDriver
##############################################################################

class PolarisDriver:


  #######################
  #  API Constants
  #######################

  # PHSR Constants
  PHSR_REPORT_ALLOCATED             = '00'
  PHSR_REPORT_NEED_FREE             = '01'
  PHSR_REPORT_NOT_INIT              = '02'
  PHSR_REPORT_NOT_ENABLED           = '03'
  PHSR_REPORT_ENABLED               = '04'

  # PENA Constants
  PENA_TTPRIO_STATIC                = 'S'
  PENA_TTPRIO_DYNAMIC               = 'D'
  PENA_TTPRIO_BBOX                  = 'B'

  # TSTART Constants
  TSTART_RESET_FRAMECOUNT           = '80'
  TSTART_NORESET                    = ''

  #######################
  #  CONSTRUCTOR
  #######################

  def __init__(self, port, timeout=0.03):
  
    # The other parameters need not be changed
    self.serial = serial.Serial()
    self.serial.port     = port 
    self.serial.timeout  = timeout

    # PortHandler
    self.port_handler = PortHandler()


  #######################
  #  PortHandler Functions
  #######################

  def init(self):
    return self._init()

  def assignPortHandleAll(self):
    self.port_handler = PortHandler(self._phsr(self.PHSR_REPORT_ALLOCATED))

  def initPortHandleAll(self):
    for handle in self.port_handler.handles:
      self._pinit(handle.id)
    self.assignPortHandleAll()

  def enablePortHandle(self, port_handle, ttpriority):
    return self._pena(port_handle.id,ttpriority)

  def startTracking(self, reply_option):
    return self._tstart(reply_option)

  def stopTracking(self):
    return self._tstop()

  def getPositionFromBX(self, reply_option):
    bx_data = self.BX(reply_option).encode('hex')
    # print bx_data
    bx_data = bx_data[42:66]

    def stringSwap(str_to_swap):
      '''Method used to swap the data endian'''
      size = len(str_to_swap)/2
      ret_str = ''
      for i in range(size):
        ret_str = str_to_swap[:2] + ret_str
        str_to_swap = str_to_swap[2:]
      return ret_str

    x_str = bx_data[:8]
    bx_data = bx_data[8:]
    y_str = bx_data[:8]
    bx_data = bx_data[8:]
    z_str = bx_data[:8]
    bx_data = bx_data[8:]

    if(len(x_str) > 7):
      x = struct.unpack("!f",stringSwap(x_str[:8]).decode('hex'))[0]
    else:
      x = 'miss'

    if(len(y_str) > 7):
      y = struct.unpack("!f",stringSwap(y_str[:8]).decode('hex'))[0]
    else:
      y = 'miss'

    if(len(z_str) > 7):
      z = struct.unpack("!f",stringSwap(z_str[:8]).decode('hex'))[0]
    else:
      z = 'miss'

    pos = [x, y, z]

    return pos

  def getToolTransformations(self):

    def stringSwap(str_to_swap):
      '''Method used to swap the data endian'''
      size = len(str_to_swap)/2
      ret_str = ''
      for i in range(size):
        ret_str = str_to_swap[:2] + ret_str
        str_to_swap = str_to_swap[2:]
      return ret_str

    string     = self._bx('')
    numhandles = int(string[:2])
    string     = string[2:]

    if numhandles != len(self.port_handler.handles):
      print '''Reply has a different number of handles than expected!
               Reassigning ports should correct this mistake. '''
      return
    else:
      for handle in self.port_handler.handles:

        # Check handle ID
        cur_handle_id = string[:2]
        string = string[2:]
        if (cur_handle_id != handle.id):
          print 'Unexpected handle id.'
        
        # Check status
        cur_handle_status = string[:2]
        string = string[2:]
        if   (cur_handle_status == handle.tool.TOOL_STATUS_VALID   ):

          # Valid 
          handle.tool.status = handle.tool.TOOL_STATUS_VALID
          print 'Tool with ID '+str(handle.id)+' is valid'

          # Quaternion
          q0 = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]
          q1 = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]
          q2 = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]
          q3 = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]

          handle.tool.updateRot(q0,q1,q2,q3)

          # Translation
          x = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]
          y = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]
          z = struct.unpack("!f",stringSwap(string[:8]).decode('hex'))[0]
          string = string[8:]

          handle.tool.updateTrans(x,y,z)

          error = stringSwap(string[:8])
          string = string[8:]

          handle.tool.updateError(error)

          port_status = stringSwap(string[:8])
          string = string[8:]

          frame_number = stringSwap(string[:8])
          string = string[8:]

          handle.tool.updateFrameNumber(frame_number)

          return x

        elif (cur_handle_status == handle.tool.TOOL_STATUS_MISSING ):
          handle.tool.status = handle.tool.TOOL_STATUS_MISSING
          print 'Tool with ID '+str(handle.id)+' is missing'
          port_status = stringSwap(string[:8])
          string = string[8:]
          frame_number = stringSwap(string[:8])
          string = string[8:]


        elif (cur_handle_status == handle.tool.TOOL_STATUS_DISABLED):
          handle.tool.status = handle.tool.TOOL_STATUS_DISABLED
          print 'Tool with ID '+str(handle.id)+' is disabled'
   

  #######################
  #  SERIAL COMM FUNCTIONS
  #######################

  def open(self):
    if self.serial.isOpen():
      print "port already opened."
    else:
      self.serial.open()
    #self.serial.flushInput()
    #self.serial.flushOutput()

  def close(self):
    if self.serial.isOpen():
      self.serial.close()
    else:
      print 'port already closed'

  def _serialReadLine(self):
    r = ''
    fullstring = ''
    while True:
      r = self.serial.read(1)
      if r == '\r':
        break
      fullstring = fullstring + r
    return fullstring

  def _sendCommandAndCheckResponse(self,command):
    self.serial.write(command)
    response = self._serialReadLine()
    self._raiseExceptionIfError(command,response)
    return response    

  #######################
  #  API COMMANDS
  #######################

  def _3d(self,port_handle,reply_option):
    command = '3D '+str(port_handle)+str(reply_option)+'\r'
    return self._sendCommandAndCheckResponse(command)

  def _apirev(self):
    command = 'APIREV \r'
    return self._sendCommandAndCheckResponse(command)

  def _beep(self,number_of_beeps):
    command = 'BEEP '+str(number_of_beeps)+'\r'
    return self._sendCommandAndCheckResponse(command)

  def _bx(self, reply_option):
    self.serial.write('BX '+str(reply_option)+'\r')

    # This method requires a different readline
    string = self.serial.read(2)
    encoded = string.encode('hex')
    # Verify if the start sequence is alright
    if encoded != 'c4a5':
      print encoded
      print 'Error in _bx function'
    # Get reply length
    string = self.serial.read(2)
    encoded = string.encode('hex')
    reply_len = int(encoded,16)
    # Get CRC header
    string = self.serial.read(2)
    # Get Response
    response = self.serial.read(reply_len).encode('hex')
    # Get CRC
    string = self.serial.read(2)
    return response

  def BX(self, reply_option):
    self.serial.write('BX '+str(reply_option)+'\r')

    start_sqn = self.serial.read(64)
    pos = self.serial.read(24)
    crc = self.serial.read(8)
    # print start_sqn + pos + crc
    return start_sqn + pos + crc

  def _comm(self, baud_rate, data_bits, parity, stop_bits, hardware_handshaking):
    self.serial.write('COMM '+str(baud_rate)+str(data_bits)+str(parity)+str(stop_bits)+str(hardware_handshaking)+'\r')
    return self._serialReadLine()

  def _dflt(self, user_parameter_name):
    self.serial.write('DFLT '+str(user_parameter_name)+'\r')
    return self._serialReadLine()

  def _dstart(self, reply_option):
    self.serial.write('DSTART '+str(reply_option)+'\r')
    return self._serialReadLine()

  def _dstop(self):
    self.serial.write('DSTOP \r')
    return self._serialReadLine()

  def _echo(self, ascii_characters):
    self.serial.write('DFLT '+str(ascii_characters)+'\r')
    return self._serialReadLine()

  def _get(self, user_parameter_name):
    self.serial.write('GET '+str(user_parameter_name)+'\r')
    return self._serialReadLine()

  def _getinfo(self, user_parameter_name):
    self.serial.write('GETINFO '+str(user_parameter_name)+'\r')
    return self._serialReadLine()

  def _getio(self):
    self.serial.write('GETIO \r')
    return self._serialReadLine()

  def _getlog(self, offset, length, logname):
    self.serial.write('GETLOG '+str(offset)+str(length)+str(logname)+'\r')
    return self._serialReadLine()

  def _hcwdog(self, timeout_value):
    self.serial.write('HCWDOG '+str(timeout_value)+'\r')
    return self._serialReadLine()

  def _init(self):
    command = 'INIT \r'
    return self._sendCommandAndCheckResponse(command)

  def _irate(self, illuminator_rate):
    self.serial.write('IRATE '+str(illuminator_rate)+'\r')
    return self._serialReadLine()

  def _ired(self, port_handle, marker_activation_signature):
    self.serial.write('IRED '+str(port_handle)+str(marker_activation_signature)+'\r')
    return self._serialReadLine()

  def _led(self, port_handle, led_number, state):
    self.serial.write('LED '+str(port_handle)+str(led_number)+str(state)+'\r')
    return self._serialReadLine()

  def _pdis(self, port_handle):
    self.serial.write('PDIS '+str(port_handle)+'\r')
    return self._serialReadLine()

  def _pena(self, port_handle, tool_tracking_priority):
    self.serial.write('PENA '+str(port_handle)+str(tool_tracking_priority)+'\r')
    return self._serialReadLine()

  def _pfsel(self, port_handle, face_selection):
    self.serial.write('PFSEL '+str(port_handle)+str(face_selection)+'\r')
    return self._serialReadLine()

  def _phf(self, port_handle):
    self.serial.write('PHF '+str(port_handle)+'\r')
    return self._serialReadLine()

  def _phinf(self, port_handle, reply_option):
    self.serial.write('PHINF '+str(port_handle)+str(reply_option)+'\r')
    return self._serialReadLine()

  def _phrq(self, hardware_device, system_type, tool_type, port_number):
    self.serial.write('PHRQ '+str(hardware_device)+str(system_type)+str(tool_type)+str(port_number)+'\r')
    return self._serialReadLine()

  def _phsr(self, reply_option):
    command = 'PHSR '+str(reply_option)+'\r'
    return self._sendCommandAndCheckResponse(command)

  def _pinit(self, port_handle):
    self.serial.write('PINIT '+str(port_handle)+'\r')
    return self._serialReadLine()

  def _pprd(self, port_handle, srom_device_address):
    self.serial.write('PPRD '+str(port_handle)+str(srom_device_address)+'\r')
    return self._serialReadLine()

  def _ppwr(self, port_handle, srom_device_address, srom_device_data):
    self.serial.write('PPWR '+str(port_handle)+str(srom_device_address)+str(srom_device_data)+'\r')
    return self._serialReadLine()

  def _psel(self, port_handle, srom_device_id):
    self.serial.write('PSEL '+str(port_handle)+str(srom_device_id)+'\r')
    return self._serialReadLine()

  def _psout(self, port_handle, gpio_1_state, gpio_2_state, gpio_3_state, gpio_4_state ):
    self.serial.write('PSOUT '+str(gpio_1_state)+str(gpio_2_state)+str(gpio_3_state)+str(gpio_4_state)+'\r')
    return self._serialReadLine()

  def _psrch(self, port_handle):
    self.serial.write('PSRCH '+str(port_handle)+'\r')
    return self._serialReadLine()

  def _purd(self, port_handle, user_srom_device_address):
    self.serial.write('PURD '+str(port_handle)+str(user_srom_device_address)+'\r')
    return self._serialReadLine()

  def _puwr(self, port_handle, user_srom_device_address, user_srom_device_data):
    self.serial.write('PUWR '+str(port_handle)+str(user_srom_device_address)+str(user_srom_device_data)+'\r')
    return self._serialReadLine()

  def _pvwr(self, port_handle, start_address, tool_definition_file_data):
    self.serial.write('PVWR '+str(port_handle)+str(start_address)+str(tool_definition_file_data)+'\r')
    return self._serialReadLine()

  def _reset(self, reset_option):
    self.serial.write('RESET '+str(reset_option)+'\r')
    return self._serialReadLine()

  def _save(self):
    self.serial.write('SAVE \r')
    return self._serialReadLine()

  def _sensel(self, option):
    self.serial.write('SENSEL '+str(option)+'\r')
    return self._serialReadLine()

  def _set(self, user_parameter_name, value):
    self.serial.write('SET '+str(user_parameter_name)+'='+str(value)+'\r')
    return self._serialReadLine()

  def _setio(self, input_output_line_status):
    self.serial.write('SETIO '+str(input_output_line_status)+'\r')
    return self._serialReadLine()

  def _sflist(self, reply_option):
    command = 'SFLIST '+str(reply_option)+'\r'
    return self._sendCommandAndCheckResponse(command)

  def _sstat(self, reply_option):
    self.serial.write('SSTAT '+str(reply_option)+'\r')
    return self._serialReadLine()

  def _syslog(self, device_name, category, message): 
    self.serial.write('SYSLOG '+str(device_name)+str(category)+'='+str(message)+'\r')
    return self._serialReadLine()

  def _tctst(self, port_handle):
    self.serial.write('TCTST '+str(port_handle)+'\r')
    return self._serialReadLine()

  def _tstart(self, reply_option):
    self.serial.write('TSTART '+str(reply_option)+'\r')
    return self._serialReadLine()

  def _tstop(self):
    self.serial.write('TSTOP \r')
    return self._serialReadLine()

  def _ttcfg(self, port_handle):
    self.serial.write('TTCFG '+str(port_handle)+'\r')
    return self._serialReadLine()

  def _tx(self, reply_option):
    self.serial.write('TX '+str(reply_option)+'\r')
    return self._serialReadLine()

  def _ver(self, reply_option):
    self.serial.write('VER '+str(reply_option)+'\r')
    return self._serialReadLine()

  def _vget(self, row, sensor, frame_index, start_column, end_column, stride):
    self.serial.write('VGET '+str(row)+str(sensor)+str(frame_index)+str(start_column)+str(end_column)+str(stride)+'\r')
    return self._serialReadLine()

  def _vsel(self, volume_number):
    self.serial.write('VSEL '+str(volume_number)+'\r')
    return self._serialReadLine()

  def _vsnap(self):
    self.serial.write('VSNAP \r')
    return self._serialReadLine()

  #######################
  #  ERROR AUX FUNCTIONS
  #######################

  def _raiseExceptionIfError(self,command,response):
    if response[:5] == 'ERROR':
      error_code = response[5:7]
      raise CommandError(command,error_code)
    elif response[:5] == 'RESET':
      raise CommandError(command,'01')
    else:
      return

  #######################
  #  HIGHER LEVEL FUNCTION
  #######################

  def initStrayMarkerTracker(self):
    verbose = False
    if verbose == True:
      print self._ver("4")
      print self._comm("7", "0", "0", "0", "1")
      print self._ver("5")
      print self._getinfo("Config.*")
      print self._get("Device.*")
      print self._init()
      print self._phsr("00")
      print self._getinfo("Param.Tracking.*")
      print self._getinfo("Features.Firmware.Version")
      print self._getinfo("Info.Status.Alerts")
      print self._getinfo("Info.Status.New Alerts")
      print self._getinfo("Features.Hardware.Serial Number")
      print self._ver("4")
      print self._getinfo("Features.Tools.*")
      print self._sflist("03")
      print self._getinfo("Param.Tracking.Selected Volume")
      print self._getinfo("SCU-0.Info.Status.New Alerts")
      print self._getinfo("SCU-0.Info.Status.Alerts")
      print self._phinf("01","0075")
      print self._getinfo("Info.Status.New Alerts")
      print self._getinfo("Info.Status.Alerts")
      print self._getinfo("STB-0.Info.Status.New Alerts")
      print self._getinfo("STB-0.Info.Status.Alerts")

      print self._tstart('80')
      print self.getPositionFromBX("1801")
      print self._tstop()

      print self._set("PS-0.Param.Tracking.Illuminator Rate","2")
      print self._phrq("********", "*", "1", "****")
      print self._pvwr("02", "0000", "4E444900D2110000010000000000000100000000031480345A00000004000000040000000000403F000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "0040", "00002041000000000000000000000000000000000000000052B8E4417B14244200000000000000000000B04200000000AE4731C2CDCC21420000000000000000") 
      print self._pvwr("02", "0080", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "00C0", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "0100", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000803F00000000") 
      print self._pvwr("02", "0140", "000000000000803F00000000000000000000803F00000000000000000000803F0000000000000000000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "0180", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "01C0", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "0200", "0000000000000000000000000000000000000000000000000000000000000000000000000000000000010203000000000000000000000000000000001F1F1F1F") 
      print self._pvwr("02", "0240", "090000004E4449000000000000000000383730303333390000000000000000000000000009010101010000000000000000000000000000000001010101000000") 
      print self._pvwr("02", "0280", "000000000000000000000000008000290000000000000000000080BF000000000000000000000000000000000000000000000000000000000000000000000000") 
      print self._pvwr("02", "02C0", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      print self._pinit("02")
      print self._phinf("02", "0075")

      print self._tstart('80')
      print self.getPositionFromBX('1803')
      print self._tstop()

      print self._pena("02", "D")

      print self._tstart('80')
    else:
      print "Restarting system..."
      self._ver("4")
      self._comm("7", "0", "0", "0", "1")
      self._ver("5")
      self._getinfo("Config.*")
      self._get("Device.*")

      print "Starting system..."
      self._init()
      self._phsr("00")
      self._getinfo("Param.Tracking.*")
      self._getinfo("Features.Firmware.Version")
      self._getinfo("Info.Status.Alerts")
      self._getinfo("Info.Status.New Alerts")
      self._getinfo("Features.Hardware.Serial Number")
      self._ver("4")
      self._getinfo("Features.Tools.*")
      self._sflist("03")
      self._getinfo("Param.Tracking.Selected Volume")
      self._getinfo("SCU-0.Info.Status.New Alerts")
      self._getinfo("SCU-0.Info.Status.Alerts")
      self._phinf("01","0075")
      self._getinfo("Info.Status.New Alerts")
      self._getinfo("Info.Status.Alerts")
      self._getinfo("STB-0.Info.Status.New Alerts")
      self._getinfo("STB-0.Info.Status.Alerts")
      
      print "Starting passive tool handle..."
      self._tstart('80')
      self.getPositionFromBX("1801")
      self._tstop()

      self._set("PS-0.Param.Tracking.Illuminator Rate","2")
      self._phrq("********", "*", "1", "****")
      self._pvwr("02", "0000", "4E444900D2110000010000000000000100000000031480345A00000004000000040000000000403F000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "0040", "00002041000000000000000000000000000000000000000052B8E4417B14244200000000000000000000B04200000000AE4731C2CDCC21420000000000000000") 
      self._pvwr("02", "0080", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "00C0", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "0100", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000803F00000000") 
      self._pvwr("02", "0140", "000000000000803F00000000000000000000803F00000000000000000000803F0000000000000000000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "0180", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "01C0", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "0200", "0000000000000000000000000000000000000000000000000000000000000000000000000000000000010203000000000000000000000000000000001F1F1F1F") 
      self._pvwr("02", "0240", "090000004E4449000000000000000000383730303333390000000000000000000000000009010101010000000000000000000000000000000001010101000000") 
      self._pvwr("02", "0280", "000000000000000000000000008000290000000000000000000080BF000000000000000000000000000000000000000000000000000000000000000000000000") 
      self._pvwr("02", "02C0", "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") 
      self._pinit("02")
      self._phinf("02", "0075") 

      print "Enabling tool..."
      self._tstart('80')
      self.getPositionFromBX('1803')
      self._tstop()

      self._pena("02", "D")

      print "Start!"
      self._tstart('80')

    return


##############################################################################
#                          API EXCEPTIONS
############################################################################## 

class CommandError(Exception):

  def __init__(self,command,value):
    self.command = command
    self.value = value
    if value == '01':
      self.msg = 'Error ' + value + ' Unexpected RESET response'
    elif value == '42':
      self.msg = 'Error ' + value + ' No device detected'
    else :
      self.msg = 'Error ' + value

  def __str__(self):
    return 'CommandError: ' + self.msg + ' when calling command ' + self.command

##############################################################################
#                          RUNNING THE MAIN ROUTINE
############################################################################## 

if __name__ == '__main__':
  main()
