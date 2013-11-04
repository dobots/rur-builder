# -*- python -*-
#                           Package   : omniidl
# android.py                Created on: 2013/10/18
#                           Author    : Bart van Vliet
#
#    Copyright (C) 2013 Almende B.V.
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
#   Android messengers for network abstractions 

"""Android IDL compiler back-end."""

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur

class Android:

	def __init__(self, st, visitor, portList):
		self.st = st
		self.vs = visitor
		self.portList = portList

##########################################################################################
#################################### Standard function ###################################
##########################################################################################

# Includes
	def writeIncludes(self):
		print "#include <vector>"
		print "#include <deque>"
		print "#include <string>"
		print

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
		self.writeAndroidReadStructs(p)

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
		self.writePortDeclaration(p)

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
		self.writePort(p)
		self.writeAndroidPort(p)

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
		pass

# Destructor implementation
	def writeDestructorImplStart(self):
		pass

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
		self.writePortImpl(p)
		self.writeAndroidPortImpl(p)



##########################################################################################
##################################### Write function #####################################
##########################################################################################



	# For every port we define what the port name will be. We used to have an interator to
	# enable multiple ports with incrementing indices, but this does make things less 
	# transparent to the user. So, if you have two microphones, you will need to create two
	# functions: 
	#   void MicrophoneLeft(in long mic_left)
	#   void MicrophoneRight(in long mic_right)
	# The ports and values are private to the class.
	def writePortDeclaration(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out(param_type + " mPort" + port_name + "Value;")
			self.st.out("std::deque<" + param_type + "> mPort" + port_name + "ReadBuf;")
		else:
			self.st.out("std::deque<" + param_type + "> mPort" + port_name + "WriteBuf;")
		self.st.out("")

	# The ports themselves will become again functions, like readInput() or writeOutput()
	# The result of this function will be a list of such functions
	# All these functions will be "protected" and can be accessed only by the class or its parent.
	def writePort(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		self.vs.writePortFunctionSignature(p)

	def writeAndroidReadStructs(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.OUT:
			self.st.out("struct Android" + port_name + "Read_t {")
			self.st.inc_indent()
			self.st.out("bool success;")
			self.st.out(param_type + " val;")
			self.st.dec_indent()
			self.st.out("};")
			self.st.out("")

	def writeAndroidPort(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("void androidWrite" + port_name + "(" + param_type + " " + param_name + ");")
		if port_direction == rur.Direction.OUT:
			self.st.out("Android" + port_name + "Read_t androidRead" + port_name + "();")
		self.st.out("")

	def writePortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.IN)
			self.st.out("if (mPort" + port_name + "ReadBuf.empty())")
			self.st.inc_indent()
			self.st.out("return NULL;")
			self.st.dec_indent()
			if param_kind == idltype.tk_sequence:
				self.st.out("mPort" + port_name + "ReadBuf.front().swap(mPort" + port_name + "Value);")
			else:
				self.st.out("mPort" + port_name + "Value = mPort" + port_name + "ReadBuf.front();")
			self.st.out("mPort" + port_name + "ReadBuf.pop_front();")
			self.st.out("return &mPort" + port_name + "Value;")
		if port_direction == rur.Direction.OUT:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.OUT)
			if param_kind == idltype.tk_sequence:
				# Make a copy
				self.st.out("mPort" + port_name + "WriteBuf.push_back(" + param_type + "(" + param_name + "));")
			else:
				# Already a copy
				self.st.out("mPort" + port_name + "WriteBuf.push_back(" + param_name + ");")
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writeAndroidPortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("void " + self.vs.classname + "::androidWrite" + port_name + "(" + param_type + " in) {")
			self.st.inc_indent()
			self.st.out("mPort" + port_name + "ReadBuf.push_back(in);")
			#if param_kind == idltype.tk_sequence:
			#else:	
		if port_direction == rur.Direction.OUT:
			self.st.out("Android" + port_name + "Read_t " + self.vs.classname + "::androidRead" + port_name + "() {")
			self.st.inc_indent()
			self.st.out("Android" + port_name + "Read_t ret;")
			self.st.out("if (mPort" + port_name + "WriteBuf.empty()) {")
			self.st.inc_indent()
			self.st.out("ret.success = false;")
			self.st.out("return ret;")
			self.vs.writeFunctionEnd()
			if param_kind == idltype.tk_sequence:
				self.st.out("mPort" + port_name + "WriteBuf.front().swap(ret.val);")
			else:
				self.st.out("ret.val = mPort" + port_name + "WriteBuf.front();")
			self.st.out("ret.success = true;")
			self.st.out("mPort" + port_name + "WriteBuf.pop_front();")
			self.st.out("return ret;")
		self.vs.writeFunctionEnd()
		self.st.out("")

