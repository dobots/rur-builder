# -*- python -*-
#                           Package   : omniidl
# main_impl.py              Created on: 2013/05/23
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


class Main_impl (Main):
	def writeAll(self):
		self.vs.writeFileComment()
		
		# INCLUDES
		self.vs.writeIncludesImpl()
		for k, b in self.beList.iteritems():
			b.writeIncludesImpl()
		self.st.out("")
		
		self.vs.writeUsingNamespace()
		for k, b in self.beList.iteritems():
			b.writeUsingNamespace()
		self.st.out("")
		
#		# BEFORE CLASS
#		self.vs.writeBeforeClassFuncsImpl()
#		for k, b in self.beList.iteritems():
#			b.writeBeforeClassFuncsImpl()
#		for p in self.vs.portList:
#			self.beList[p.backend].writeBeforeClassPortFuncsImpl(p)
		
		# CONSTRUCTOR
		self.vs.writeConstructorImplStart() # Should be self.be ?
		self.st.inc_indent()
		# var initialization
		for k, b in self.beList.iteritems():
			b.writeConstructorImplInit()
		for p in self.vs.portList:
			self.beList[p.beStr].writeConstructorImplPortInit(p)
		self.vs.writeConstructorImplInit()
		self.st.dec_indent()
		self.st.out("{")
		self.st.inc_indent()
		# actual implementation
		self.vs.writeConstructorImpl()
		for k, b in self.beList.iteritems():
			b.writeConstructorImpl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeConstructorImplPort(p)
		self.vs.writeFunctionEnd()
		self.st.out("")
		
		# DESTRUCTOR
		self.vs.writeDestructorImplStart() # Should be self.be ?
		self.st.inc_indent()
		self.vs.writeDestructorImpl()
		for k, b in self.beList.iteritems():
			b.writeDestructorImpl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeDestructorImplPort(p)
		self.vs.writeFunctionEnd()
		self.st.out("")
		
		# INIT
		self.vs.writeInitImplStart()
		self.st.inc_indent()
		self.vs.writeInitImpl()
		for k, b in self.beList.iteritems():
			b.writeInitImpl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeInitImplPort(p)
		self.vs.writeFunctionEnd()
		self.st.out("")
		
		# IMPLEMENTATION
		self.vs.writeFuncsImpl()
		for k, b in self.beList.iteritems():
			b.writeFuncsImpl()
		for p in self.vs.portList:
			self.beList[p.beStr].writePortFuncsImpl(p)
		
		print "} // namespace"

# Initialize this parser
def run(tree, args):
	st = output.Stream(sys.stdout, 2)
	main = Main_impl(st, tree, args)
	
	# And then write everything to the to-be-generated source file
	main.writeAll()

