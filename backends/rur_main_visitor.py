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


class MainVisitor (rur.RurModule):

	def __init__(self, st):
		self.st = st
		

##########################################################################################
#################################### Standard function ###################################
##########################################################################################

# Includes
	def writeIncludes(self):
		print "#include <string>"
		print "#include <vector>" # almost always needed

	def writeIncludesImpl(self):
		print "#include \"" + self.classname + ".h\""

	def writeUsingNamespace(self):
#		print "using namespace " + self.namespace + ";"
		print "namespace " + self.namespace + " {"

# Before class
	def writeBeforeClassVarsDecl(self):
		self.writeStructDeclarations()
		self.writeTypedefs()

	def writeBeforeClassFuncsDecl(self):
		pass

	def writeBeforeClassPortVarsDecl(self, p):
		pass

	def writeBeforeClassPortFuncsDecl(self, p):
		pass

# Private
	def writePrivateVarsDecl(self):
		self.writeStructInstances()

	def writePrivateFuncsDecl(self):
		pass

	def writePrivatePortVarsDecl(self, p):
		pass

	def writePrivatePortFuncsDecl(self, p):
		pass

# Protected
	def writeProtectedVarsDecl(self):
		pass

	def writeProtectedFuncsDecl(self):
		self.writePortsAsArray()

	def writeProtectedPortVarsDecl(self, p):
		pass

	def writeProtectedPortFuncsDecl(self, p):
		pass

# Public
	def writePublicVarsDecl(self):
		pass

	def writePublicFuncsDecl(self):
		self.writeConstructor()
		self.writeDestructor()
		self.writeInit()
		self.writeGetParam()
		self.writeTick()
		self.writeStop()

	def writePublicPortVarsDecl(self, p):
		pass

	def writePublicPortFuncsDecl(self, p):
		pass

# Constructor implementation
	def writeConstructorImplStart(self):
		self.st.out(self.classname + "::" + self.classname + "():")

	def writeConstructorImplInit(self):
		self.st.out("cliParam(0)") # Dummy initiation to avoid compile error

	def writeConstructorImplPortInit(self, p):
		pass

	def writeConstructorImpl(self):
		self.writePortsAsArrayInit()
		self.writeStructAllocations()

	def writeConstructorImplPort(self, p):
		pass

# Destructor implementation
	def writeDestructorImplStart(self):
		self.st.out(self.classname + "::~" + self.classname + "() {")

	def writeDestructorImpl(self):
		for s in self.structList:
			name = s.scopedName()[-1]
			if name == 'Param':
				self.st.out("delete cliParam;")

	def writeDestructorImplPort(self, p):
		pass 

# Init implementation
	def writeInitImplStart(self):
		self.st.out("void " + self.classname + "::Init(std::string & name) {")

	def writeInitImpl(self):
		self.st.out("cliParam->module_id = name;") # TODO: this should be dynamical, as in writeStructDeclarations()
		self.st.out("")

	def writeInitImplPort(self, p):
		pass

# Implementation
	def writeFuncsImpl(self):
		pass

	def writePortFuncsImpl(self, p):
		pass





##########################################################################################
##################################### Write function #####################################
##########################################################################################

	def writeNamespaceStart(self):
		self.st.out("namespace " + self.namespace + " {")
		self.st.out("")

	def writeNamespaceEnd(self):
		self.st.out("} // End of namespace")
		self.st.out("")

	# The struct definition 
	def writeStructDeclarations(self):
		for s in self.structList:
			self.st.out("struct " + s.identifier() + " {")
			self.st.inc_indent()
			for m in s.members():
				if m.constrType():
					m.memberType().decl().accept(self)
				t = self.getType(m.memberType())
				for d in m.declarators():
					membername = d.identifier()
					self.st.out(t + " " + membername + ";")
			self.st.dec_indent()
			self.st.out("};")
			self.st.out("")

	# Declare internal dummy instances for custom structs
	# we define an exception for the struct Param that can be used to store command-line parameters
	def writeStructInstances(self):
		for s in self.structList: 
			name = s.scopedName()[-1]
			if name == 'Param':
				self.st.out(name + ' *cli' + name + ";")
				self.st.out("")
			else:
				self.st.out(name + ' *dummy' + name + ";")
				self.st.out("")

	# Allocate structs
	# we define an exception for the struct Param that can be used to store command-line parameters    
	def writeStructAllocations(self):
		for s in self.structList:
			name = s.scopedName()[-1]
			if name == 'Param':
				self.st.out("cli" + name + " = new " + name + "();")
			else:
				self.st.out("dummy" + name + " = new " + name + "();")

	# Only accept typedef for sequences for now
	def writeTypedefs(self):
		for t in self.typedefList:
			for d in t.declarators():
				a = t.aliasType()
				if isinstance(a,idltype.Sequence):
					seq_type = self.seqToVec(a)
					self.st.out("typedef " + seq_type + " " + d.identifier() + ";")
				self.st.out("")

	def writePortsAsArray(self):
		names = [];
		for p in self.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.getPortConfiguration(p)
			if port_direction == rur.Direction.IN:
				  names.append("\"read" + port_name + "\"") 
			
			if port_direction == rur.Direction.OUT: 
				  names.append("\"write" + port_name + "\"") 
		
		self.st.out("static const int channel_count = " + str(len(names)) + ";")
		# bug: http://stackoverflow.com/questions/9900242/error-with-constexprgcc-error-a-brace-enclosed-initializer-is-not-allowed-h
		# self.st.out("const char* const channel[" + str(len(names)) + "] = {" + ', '.join(names) + "};")
		self.st.out("const char* channel[" + str(len(names)) + "];")

	def writePortsAsArrayInit(self):
		names = [];
		for p in self.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.getPortConfiguration(p)
			if port_direction == rur.Direction.IN:
				names.append("\"read" + port_name + "\"") 
			if port_direction == rur.Direction.OUT: 
				names.append("\"write" + port_name + "\"") 
		# bug: http://stackoverflow.com/questions/9900242/error-with-constexprgcc-error-a-brace-enclosed-initializer-is-not-allowed-h
		self.st.out("const char* const channel[" + str(len(names)) + "] = {" + ', '.join(names) + "};")

	def writeConstructor(self):
		self.st.out(self.classname + "();")
		self.st.out("")

	def writeDestructor(self):
		self.st.out("~" + self.classname + "();")
		self.st.out("")

	def writeInit(self):
		self.st.out("// Extend this with your own code, first call " + self.classname + "::Init(name);")
		self.st.out("void Init(std::string& name);")
		self.st.out("")

	def writeGetParam(self):
		# Dedicate setter and getter for Param struct
		for s in self.structList:
			name = s.scopedName()[-1]
			if name == 'Param':
				self.st.out("// Function to get Param struct (to subsequently set CLI parameters)")
				self.st.out("inline Param *GetParam() { return cliParam; }")
				self.st.out("")

	def writeTick(self):
		self.st.out("// Overwrite this function with your own code")
		self.st.out("virtual void Tick() {}")
		self.st.out("")

	def writeStop(self):
		self.st.out("// Overwrite this function with your own code")
		self.st.out("bool Stop() { return false; }")
		self.st.out("")
