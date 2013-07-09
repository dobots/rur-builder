# -*- python -*-
#                           Package   : omniidl
# standard.py               Created on: 2011/09/30
#                           Author    : Anne C. van Rossum
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

class Standard:

	def __init__(self, st, visitor, portList):
		self.st = st
		self.vs = visitor
		self.portList = portList

##########################################################################################
#################################### Standard function ###################################
##########################################################################################

# Includes
	def writeIncludes(self):
		print "#include <string>"
		print "#include <vector>" # almost always needed

	def writeIncludesImpl(self):
		pass

	def writeUsingNamespace(self):
		pass

# Before class
	def writeBeforeClassVarsDecl(self):
		pass

	def writeBeforeClassFuncsDecl(self):
		pass

	def writeBeforeClassPortVarsDecl(self, p):
		pass

	def writeBeforeClassPortFuncsDecl(self, p):
		pass

	def writeClassStart(self):
		self.st.out("class " + self.vs.classname + " {")

# Private
	def writePrivateVarsDecl(self):
		pass

	def writePrivateFuncsDecl(self):
		pass

	def writePrivatePortVarsDecl(self, p):
		self.writeDummyAllocation(p)

	def writePrivatePortFuncsDecl(self, p):
		pass

# Protected
	def writeProtectedVarsDecl(self):
		pass

	def writeProtectedFuncsDecl(self):
		pass

	def writeProtectedPortVarsDecl(self, p):
		pass

	def writeProtectedPortFuncsDecl(self, p):
		pass

# Public
	def writePublicVarsDecl(self):
		pass

	def writePublicFuncsDecl(self):
		pass

	def writePublicPortVarsDecl(self, p):
		pass

	def writePublicPortFuncsDecl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
#		if port_direction == rur.Direction.IN:
#			self.vs.writePortFunctionSignature(p)
#		if port_direction == rur.Direction.OUT: 
#			self.vs.writePortFunctionSignature(p)
		self.vs.writePortFunctionSignature(p)

# Constructor implementation
	def writeConstructorImplStart(self):
		self.st.out(self.classname + "::" + self.classname + "():")

	def writeConstructorImplInit(self):
		pass

	def writeConstructorImplPortInit(self, p):
		pass

	def writeConstructorImpl(self):
		pass

	def writeConstructorImplPort(self, p):
		self.writeDummyInitiation(p)

# Destructor implementation
	def writeDestructorImplStart(self):
		self.st.out(self.classname + "::~" + self.classname + "() {")

	def writeDestructorImpl(self):
		pass

	def writeDestructorImplPort(self, p):
		pass

# Init implementation
	def writeInitImplStart(self):
		self.st.out("void " + self.classname + "::Init(std::string & name) {")

	def writeInitImpl(self):
		pass

	def writeInitImplPort(self, p):
		pass

# Implementation
	def writeFuncsImpl(self):
		pass

	def writePortFuncsImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.IN)
			self.st.out("return &dummy" + port_name + ";")
			self.vs.writeFunctionEnd()
			self.st.out("")
		if port_direction == rur.Direction.OUT: 
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.OUT)
			self.st.out("return true;")
			self.vs.writeFunctionEnd()
			self.st.out("")


##########################################################################################
##################################### Write function #####################################
##########################################################################################

	def writeDummyAllocation(self, node):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(node)
		if port_direction == rur.Direction.IN:
			self.st.out(param_type + " dummy" + port_name + ";")

	def writeDummyInitiation(self, node):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(node)
		if port_direction == rur.Direction.IN:
			self.st.out("dummy" + port_name + " = " + param_type + "(0);")
