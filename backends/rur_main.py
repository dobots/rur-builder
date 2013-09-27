# -*- python -*-
#                           Package   : omniidl
# main.py                   Created on: 2013/05/23
#                           Author    : B. van vliet
#
#    Copyright (C) 2011 Almende B.V.
#
#  This file is part of omniidl.
#
#  omniidl is free software; you can redistribute it and/or modify it
#  under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
#  02111-1307, USA.
#
# Description:
#   
#   Standard back-end for network abstractions 

"""Standard IDL compiler back-end."""

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur
from rur_main_visitor import MainVisitor

# Actual backends
from standard import Standard
from yarp import Yarp
from nodejs import NodeJS
from zeromq import ZeroMQ
from ros import Ros

class Main:
	def __init__(self, st, tree, args):
		self.st = st
		self.vs = MainVisitor(st)
		tree.accept(self.vs)
		
		if (len(args) > 0):
			beStrDef = args[0]
		else:
			beStrDef = "standard"
		
		# Keep up a port list for each backend
		bePortLists = {beStrDef: []}
		
		for p in self.vs.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
			beStr = ""
			for pr in port_comments:
				#print(pr)
				if (pr.split(" ")[1] == "middleware"):
					beStr = string.strip(pr.split(" ")[2]) # Strip trailing and leading whitespace
					break
			if (beStr == ""):
				beStr = beStrDef # No specific backend for this port
			
			if (beStr in bePortLists):
				bePortLists[beStr].append(p)
			else:
				bePortLists[beStr] = [p]
			p.beStr = beStr
		
		# Create a list of backends that are used
		self.beList = {}
		for beStr in bePortLists:
			self.beList[beStr] = self.getBackend(beStr, bePortLists[beStr])
		
		# Store the default backend
		self.beStrDef = beStrDef

	def getBackend(self, string, portList):
		if (string == "standard"):
			return Standard(self.st, self.vs, portList)
		elif (string == "yarp"):
			return Yarp(self.st, self.vs, portList)
		elif (string == "zeromq"):
			return ZeroMQ(self.st, self.vs, portList)
		elif (string == "nodejs"):
			return NodeJS(self.st, self.vs, portList)
		elif (string == "ros"):
			return Ros(self.st, self.vs, portList)
		else:
			raise Exception("Invalid backend: \"" + string +"\"")
