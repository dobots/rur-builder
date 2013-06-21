# -*- python -*-
#                           Package   : omniidl
# main_head.py              Created on: 2013/05/23
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
from rur_main import Main

class Main_head (Main):
	def writeAll(self):
		for p in self.vs.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
			channel = port_name.lower()
			if port_direction == rur.Direction.IN:
				self.st.out('in ' + channel + '0')
			if port_direction == rur.Direction.OUT:
				self.st.out('out ' + channel + '0')


# Initialize this parser
def run(tree, args):
	st = output.Stream(sys.stdout, 2)
	main = Main_head(st, tree, args)
	
	# And then write everything to the to-be-generated header file
	main.writeAll()

