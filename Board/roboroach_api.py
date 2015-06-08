#!/usr/bin/env python

"""
RoboRoach API for Ubuntu 14.04 LTS

# Copyright (c) 2015
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
from gattlib import GATTRequester
import time

##############################################################################
#                                AUTHORSHIP
##############################################################################

__author__  = "Lucas de Levy O."
__email__   = "lucasdelevy@lara.unb.br"
__license__ = "LGPL"

##############################################################################
#                            MAIN ROUTINE (FOR TESTING)
##############################################################################

def main():
  # Initialize class object
  roboroach = RoboRoach(mac_address="90:59:AF:14:08:E8")

  print("Turning right...")
  roboroach._turn("right");

  time.sleep(1)

  print("Turning left...")
  roboroach._turn("left")

##############################################################################
#                            CLASS RoboRoach
##############################################################################

class RoboRoach:

  #######################
  #  API Constants
  #######################

  ROBOROACH_FREQUENCY_HANDLE	=	0x002A
  ROBOROACH_PULSE_WIDTH			  =	0x002D
  ROBOROACH_NUM_PULSES		  	=	0x0030
  ROBOROACH_RANDOM_MODE       =	0x0033
  ROBOROACH_RIGHT_HANDLE		  =	0x0036
  ROBOROACH_LEFT_HANDLE		   	=	0x0039
  ROBOROACH_GAIN			      	=	0x003C
  ROBOROACH_FREQ_MIN		    	=	0x003F
  ROBOROACH_FREQ_MAX			    =	0x0042
  ROBOROACH_PW_MIN			     	=	0x0045
  ROBOROACH_PW_MAX				    =	0x0048
  ROBOROACH_GAIN_MIN          = 0x004B
  ROBOROACH_GAIN_MAX          = 0x004E

  #######################
  #  CONSTRUCTOR
  #######################

  def __init__(self, mac_address):
  	self.mac_address = mac_address

  	self.req = GATTRequester(mac_address)
  	print type(self.req)

  #######################
  #  COMMON FUNCTIONS
  #######################

  def _turn(self, direction):
  	if direction == 'right':
  	  self.req.write_by_handle(self.ROBOROACH_LEFT_HANDLE, str(bytearray([1])))
  	elif direction == 'left':
	  self.req.write_by_handle(self.ROBOROACH_RIGHT_HANDLE, str(bytearray([1])))
  	else:
  	  print "Unknown direction"


if __name__ == '__main__':
  main()