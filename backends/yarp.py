# -*- python -*-
#                           Package   : omniidl
# yarp.py                   Created on: 2011/09/30
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
#   YARP back-end for network abstractions 

"""YARP IDL compiler back-end."""

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur

# TODO: finish coversion of self.writePort
# TODO: Close() should this be a default function?
# TODO: port->writeStrict() and port->setStrict()

class Yarp:

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
		print "#include <vector>"
		print "#include <sstream>"
		print "#include <yarp/os/BufferedPort.h>"
		print "#include <yarp/os/Network.h>"
		print "#include <yarp/os/Bottle.h>"

	def writeIncludesImpl(self):
		pass

	def writeUsingNamespace(self):
		print "using namespace yarp::os;"

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
		self.st.out("yarp::os::Network yarp;")

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
		self.writePort(p)

# Public
	def writePublicVarsDecl(self):
		pass

	def writePublicFuncsDecl(self):
		pass

	def writePublicPortVarsDecl(self, p):
		pass

	def writePublicPortFuncsDecl(self, p):
		pass

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
		self.writePortAllocation(p)

# Destructor implementation
	def writeDestructorImplStart(self):
		pass

	def writeDestructorImpl(self):
		pass

	def writeDestructorImplPort(self, p):
		self.writePortDestruction(p)

# Init implementation
	def writeInitImplStart(self):
		self.st.out("void " + self.classname + "::Init(std::string & name) {")

	def writeInitImpl(self):
		self.writePortInit()

	def writeInitImplPort(self, p):
		self.writePortInitPort(p)

# Implementation
	def writeFuncsImpl(self):
		pass

	def writePortFuncsImpl(self, p):
		self.writePortImpl(p)


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
			self.st.out(param_type + " " + port + "Buf;")
			
#			if param_kind == idltype.tk_sequence:
#				#self.st.out("// private storage for " + port_name + "Values;")
#				#self.st.out("std::vector<" + seq_type + "> *" + port_name + "Values;")
#				self.st.out(param_type + " *" + port + "Values;")
#			else:
#				#self.st.out("// private storage for " + port_name + "Value")
#				self.st.out("int " + port + "Value;")
#		#self.st.out("// the port " + port_name + " itself") 
		self.st.out("yarp::os::BufferedPort<yarp::os::Bottle> *" + port + ";")

	# In the constructor we allocate the port, most often we will need a new BufferedPort with a 
	# Bottle as parameter. In case of a sequence we need to allocate a corresponding vector
	def writePortAllocation(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
#		if param_kind == idltype.tk_sequence:
#			if port_direction == rur.Direction.IN:
#				self.st.out(port + "Values = new " + param_type + "();")
#			param_type = "Bottle"
#		else:
#			param_type = "Bottle"
#		self.st.out(port + " = new BufferedPort<" + param_type + ">();")
		self.st.out(port + " = new BufferedPort<Bottle>();")

	# In the constructor we allocate the port, most often we will need a new BufferedPort with a 
	# Bottle as parameter. In case of a sequence we need to allocate a corresponding vector
	def writePortDestruction(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
#		if param_kind == idltype.tk_sequence:
#			if port_direction == rur.Direction.IN:
#				self.st.out("delete " + port + "Values;")
		self.st.out("delete " + port + ";")

	# In the Init() routine we open the port and if necessary set the default values of the corresponding
	# data structures 
	def writePortInit(self):
		self.st.out("std::stringstream yarpPortName;")

	def writePortInitPort(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		self.st.out("yarpPortName.str(\"\"); yarpPortName.clear();")
		#self.st.out("std::stringstream portName; portName.str(""); portName.clear();")
		self.st.out("yarpPortName << \"/" + self.vs.classname.lower() + "\" << name << \"/" + port_name.lower() + "\";") # << " + itIndex + ";")
		self.st.out(port + "->open(yarpPortName.str().c_str());")
		self.st.out("")

	# The ports themselves will become again functions, like readInput() or writeOutput()
	# The result of this function will be a list of such functions
	# All these functions will be "protected" and can be accessed only by the class or its parent.
	def writePort(self, p):
#		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
#		if port_direction == rur.Direction.IN:
#			self.vs.writePortFunctionSignature(p)
#		if port_direction == rur.Direction.OUT: 
#			self.vs.writePortFunctionSignature(p)
		self.vs.writePortFunctionSignature(p)

	def writePortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		# We have YARP specific data types
		if param_kind == idltype.tk_sequence:
			seq_type = self.vs.getSeqType(param_type)
		else:
			seq_type = param_type
		if seq_type == "int":
			capValue = "Int"
		elif seq_type == "float":
			capValue = "Double" # floats will be communicated as doubles
		elif seq_type == "double":
			capValue = "Double"
		if port_direction == rur.Direction.IN:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.IN)
			if param_kind == idltype.tk_sequence:
				self.st.out("Bottle *b = " + port + "->read(blocking);")
				self.st.out("if (b != NULL) {")
				self.st.inc_indent()
				self.st.out("for (int i = 0; i < b->size(); ++i) {")
				self.st.inc_indent()
				self.st.out(port + "Buf.push_back(b->get(i).as" + capValue + "());")
				self.vs.writeFunctionEnd()
				self.vs.writeFunctionEnd()
				self.st.out("return &" + port + "Buf;")
			else:
				self.st.out("Bottle *b = " + port + "->read(blocking);") 
				self.st.out("if (b != NULL) { ")
				self.st.inc_indent()
				self.st.out(port + "Buf = b->get(0).as" + capValue + "();") 
				self.st.out("return &" + port + "Buf;") 
				self.vs.writeFunctionEnd()
				self.st.out("return NULL;")
			#else:
			#	self.st.out("return " + port_name + "[index]->read(blocking);")
			self.vs.writeFunctionEnd()
			self.st.out("")
		
		if port_direction == rur.Direction.OUT:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.OUT)
			if param_kind == idltype.tk_sequence:
				self.st.out("Bottle &" + param_name + "Prepare = " + port + "->prepare();")
				self.st.out(param_name + "Prepare.clear();")
				self.st.out("for (int i = 0; i < " + param_name + ".size(); ++i) {")
				self.st.inc_indent()
				self.st.out(param_name + "Prepare.add" + capValue + "(" + param_name + "[i]);")
				self.vs.writeFunctionEnd()
			else:
				self.st.out("Bottle &" + param_name + "Prepare = " + port + "->prepare();")
				self.st.out(param_name + "Prepare.clear();")
				self.st.out(param_name + "Prepare.add" + capValue + "(" + param_name + ");")
#			else:
#				self.st.out( param_type + "& " + param_name + "Prepare = " + portname + "->prepare();")
			self.st.out("bool forceStrict = true; // wait till previous sends are complete")
			self.st.out(port + "->write(forceStrict);")
			self.st.out("return true;")
			self.vs.writeFunctionEnd()
			self.st.out("")
