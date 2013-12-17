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
			seq_type = self.getFullType(param_type, param_kind)
			if port_direction == rur.Direction.IN:
				self.st.out('in ' + channel + ' ' + p.beStr + ' ' + seq_type)
			if port_direction == rur.Direction.OUT:
				self.st.out('out ' + channel + ' ' + p.beStr + ' ' + seq_type)

	# Get the full type, including sequence information
	def getFullType(self, param_type, param_kind):
		if param_kind == idltype.tk_sequence:
			seq_type = self.vs.getSeqType(param_type) + 'array'
		elif param_kind == idltype.tk_string:
			seq_type = "string"
		else:
			seq_type = param_type
		return seq_type;

# Initialize this parser
def run(tree, args):
	st = output.Stream(sys.stdout, 2)
	main = Main_head(st, tree, args)
	
	# And then write everything to the to-be-generated header file
	main.writeAll()

