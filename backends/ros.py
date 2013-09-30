# -*- python -*-
#                           Package   : omniidl
# ros.py                   Created on: 2011/09/30
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
#   ROS back-end for network abstractions 

"""ROS IDL compiler back-end."""

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur

# TODO: generate structs from .msg files


class Ros:

	def __init__(self, st, visitor, portList):
		self.st = st
		self.vs = visitor
		self.portList = portList

##########################################################################################
#################################### Standard function ###################################
##########################################################################################

# Includes
	
	def writeIncludes(self):
		print "#include <deque>"
		print "#include <pthread.h>"
		print "// ros specific headers"
		print "#include <ros/ros.h>"

 		print "#include <std_msgs/Float64MultiArray.h>"
# 		print "#include <std_msgs/Duration.h>"
# 		print "#include <std_msgs/String.h>"
# 		print "#include <std_msgs/Byte.h>"
# 		print "#include <std_msgs/trait_macstd_msgs.h>"
# 		print "#include <std_msgs/UInt8.h>"
# 		print "#include <std_msgs/MultiArrayLayout.h>"
# 		print "#include <std_msgs/Header.h>"
# 		print "#include <std_msgs/Int32MultiArray.h>"
 		print "#include <std_msgs/Int16.h>"
# 		print "#include <std_msgs/Bool.h>"
# 		print "#include <std_msgs/Char.h>"
 		print "#include <std_msgs/Int16MultiArray.h>"
# 		print "#include <std_msgs/UInt16MultiArray.h>"
 		print "#include <std_msgs/Float32MultiArray.h>"
 		print "#include <std_msgs/Int32.h>"
# 		print "#include <std_msgs/Int8MultiArray.h>"
# 		print "#include <std_msgs/ColorRGBA.h>"
# 		print "#include <std_msgs/Int64MultiArray.h>"
# 		print "#include <std_msgs/Int64.h>"
 		print "#include <std_msgs/UInt32MultiArray.h>"
# 		print "#include <std_msgs/Int8.h>"
# 		print "#include <std_msgs/MultiArrayDimension.h>"
# 		print "#include <std_msgs/UInt64MultiArray.h>"
# 		print "#include <std_msgs/UInt32.h>"
# 		print "#include <std_msgs/UInt16.h>"
# 		print "#include <std_msgs/UInt8MultiArray.h>"
# 		print "#include <std_msgs/ByteMultiArray.h>"
 		print "#include <std_msgs/Float64.h>"
# 		print "#include <std_msgs/Time.h>"
# 		print "#include <std_msgs/Empty.h>"
#  		print "#include <std_msgs/UInt64.h>"
 		print "#include <std_msgs/Float32.h>"

# 		print "#include \"" + self.vs.classname + "/RurInt16.h\""
# 		print "#include \"" + self.vs.classname + "/RurInt32.h\""
# 		print "#include \"" + self.vs.classname + "/RurFloat32.h\""
# 		print "#include \"" + self.vs.classname + "/RurFloat64.h\""
# 		print "#include \"" + self.vs.classname + "/RurString.h\""
# 		print "#include \"" + self.vs.classname + "/RurInt16Seq.h\""
# 		print "#include \"" + self.vs.classname + "/RurInt32Seq.h\""
# 		print "#include \"" + self.vs.classname + "/RurFloat32Seq.h\""
# 		print "#include \"" + self.vs.classname + "/RurFloat64Seq.h\""
# 		print "#include \"" + self.vs.classname + "/RurStringSeq.h\""
	
	def writeIncludesImpl(self):
		pass
		#print "#include <" + self.vs.classname + "Ext.h>"

	def writeUsingNamespace(self):
		pass
		#self.st.out("using namespace ...;")

# Class
	
	def writeClassStart(self):
		self.st.out("class " + self.vs.classname + " {")

# Before class
	
	def writeBeforeClassVarsDecl(self):
		pass

	def writeBeforeClassFuncsDecl(self):
		pass

	def writeBeforeClassPortVarsDecl(self, p):
		pass

	def writeBeforeClassPortFuncsDecl(self, p):
		pass

# Private
	
	def writePrivateVarsDecl(self):
		pass

	def writePrivateFuncsDecl(self):
		pass

	def writePrivatePortVarsDecl(self, p):
		self.writePortAllocation(p)

	def writePrivatePortFuncsDecl(self, p):
		self.writeReadCB(p)
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
		self.st.out(self.vs.classname + "::" + self.vs.classname + "():")

	def writeConstructorImplInit(self):
		pass

	def writeConstructorImplPortInit(self, p):
		self.writePortVarCtorInit(p)
		pass

	def writeConstructorImpl(self):
		#self.st.out("ros::NodeHandle rosHandle;")
		#self.st.out("ros::NodeHandle rosPrivHandle(\"~\");")
		pass

	def writeConstructorImplPort(self, p):
		self.writePortAllocationImpl(p)
		pass

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
		self.st.out("std::string nodeName = \"" + self.vs.classname.lower() + "\" + cliParam->module_id;")
		self.st.out("int argc(0);")
		self.st.out("char** argv(NULL);")
		self.st.out("ros::init(argc, argv, nodeName, ros::init_options::NoSigintHandler);")
#		self.st.out("ros::init(1, NULL, nodeName);")
		self.st.out("ros::NodeHandle rosHandle;")
		self.st.out("ros::NodeHandle rosPrivHandle(\"~\");")
#		self.st.out("std::string portName;")
		

	def writeInitImplPort(self, p):
		self.writePortInitImpl(p)

# Implementation
	
	def writeFuncsImpl(self):
		pass

	def writePortFuncsImpl(self, p):
		self.writePortImpl(p)
		self.writeReadCBImpl(p)



##########################################################################################
##################################### Write function #####################################
##########################################################################################


	# We need a vector of ports or publishers or whatever is used by the middleware
	# inputs are read by "subscribe", and written by "advertise" and "publish"  
	
	def writePortAllocation(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("ros::Subscriber " + port + "Sub;")
			#self.st.out("std::vector< ros::Subscriber > " + portname + ";")
			self.st.out("std::deque<" + param_type + "> " + port + "Buf;")
			self.st.out(param_type + " " + port + "Val;")
			self.st.out("pthread_mutex_t " + port + "Mutex;")
		if port_direction == rur.Direction.OUT:
			#self.st.out("std::vector< ros::Publisher > " + portname + ";")
			self.st.out("ros::Publisher " + port + "Pub;")
		self.st.out("")


	def writePortInitImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			#self.st.out("portName = \"" + port_name + "\" + cliParam->module_id;")
			# What is this last ,i ?
			#self.st.out(port + "Sub = rosHandle.advertise< >(portName.c_str(), 1000, boost::bind(&" + self.vs.classname + "::" + port_name + "CB, this, _1, i));")
			#self.st.out(port + "Sub = rosHandle.advertise< >(portName.c_str(), 1000, boost::bind(&" + self.vs.classname + "::" + port_name + "CB, this, _1));")
			self.st.out(port + "Sub = rosHandle.subscribe< " + self.getRosMsgType(param_type, param_kind) + ">(\"" + port.lower() + "\", 1000, boost::bind(&" + self.vs.classname + "::" + port + "CB, this, _1));")
		if port_direction == rur.Direction.OUT:
			#self.st.out("portName = \"" + port_name + "\" + cliParam->module_id;")
			#self.st.out(port + "Pub = rosHandle.advertise<" + self.getRosMsgType(param_type, param_kind) + ">(portName.c_str(), 1000);")
			self.st.out(port + "Pub = rosPrivHandle.advertise< " + self.getRosMsgType(param_type, param_kind) + ">(\"" + port.lower() + "\", 1000);")

	def writePort(self, p):
		self.vs.writePortFunctionSignature(p)

	def writeReadCB(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
# 			self.st.out("void " + port + "CB(const " + self.getRosMsgType(param_type, param_kind) + "::ConstPtr& msg);")
 			self.st.out("void " + port + "CB(const " + self.getRosMsgType(param_type, param_kind) + "::ConstPtr& msg);")

	def writeReadCBImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("void " + self.vs.classname + "::" + port + "CB(const " + self.getRosMsgType(param_type, param_kind) + "::ConstPtr& msg) {")
			self.st.inc_indent()
			self.st.out("pthread_mutex_lock(&" + port + "Mutex);")
			if (param_kind == idltype.tk_sequence):
				self.st.out(param_type + " read;")
				self.st.out(port + "Buf.push_back(read);")
				#self.st.out(self.getRosMsgType(param_type, param_kind) + "::const_iterator it;")
				self.st.out(self.getRosMsgIterType(param_type, param_kind) + " it;")
				#======================DEBUG=========================================
				# self.st.out("std::cout << msg->data.size() << std::endl;")
				#===============================================================
				self.st.out("for (it=msg->data.begin(); it!=msg->data.end(); ++it) {")
				self.st.inc_indent()
				#self.st.out("read.push_back(*it);")
				self.st.out(port + "Buf.back().push_back(*it);")
				self.st.out("}")
				self.st.dec_indent()
			else:
				self.st.out(port + "Buf.push_back(msg->data);")
			self.st.out("pthread_mutex_unlock(&" + port + "Mutex);")
			self.vs.writeFunctionEnd()
			self.st.out("")

	def writePortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.IN)
			#========================This was forgotten=====================
			self.st.out("ros::spinOnce();")
			#===============================================================
			self.st.out("pthread_mutex_lock(&" + port + "Mutex);")
			if (param_kind == idltype.tk_sequence):
				self.st.out("if (!" + port + "Buf.empty()) {")
				self.st.inc_indent()
				self.st.out(port + "Val.swap(" + port + "Buf.front());")
				self.st.out(port + "Buf.pop_front();")
				self.vs.writeFunctionEnd()
				self.st.out("pthread_mutex_unlock(&" + port + "Mutex);")
				self.st.out("return &" + port + "Val;")
			else:
				self.st.out("if (" + port + "Buf.empty()) {")
				self.st.inc_indent()
				self.st.out("pthread_mutex_unlock(&" + port + "Mutex); // Don't forget to unlock!")
				self.st.out("return NULL;")
				self.vs.writeFunctionEnd()
				self.st.out(port + "Val = " + port + "Buf.front();")
				self.st.out(port + "Buf.pop_front();")
				self.st.out("pthread_mutex_unlock(&" + port + "Mutex);")
				self.st.out("return &" + port + "Val;")
			self.vs.writeFunctionEnd()
			self.st.out("")
		if port_direction == rur.Direction.OUT:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.OUT)
			self.st.out(self.getRosMsgType(param_type, param_kind) + " msg;")
			if param_kind == idltype.tk_sequence:
				self.st.out(self.getRosMsgSeqType(param_type, param_kind) + " data;")
				self.st.out(param_type + "::const_iterator it;")
				self.st.out("for (it=" + param_name + ".begin(); it!=" + param_name + ".end(); ++it) {")
				self.st.inc_indent()
				self.st.out("data.data = *it;")
				self.st.out("msg.data.push_back(data);")
				self.vs.writeFunctionEnd()
			else:
				self.st.out("msg.data = " + param_name + ";")
			self.st.out(port + "Pub.publish(msg);")
			self.st.out("return true;")
			self.vs.writeFunctionEnd()
			self.st.out("")

	def writePortVarCtorInit(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out(port + "Buf(0),")
			self.st.out(port + "Val(0),")

	def writePortAllocationImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("pthread_mutex_init(&" + port + "Mutex, NULL);")

	def getRosMsgSeqType(self, param_type, param_kind):
		if param_kind == idltype.tk_sequence:
			seq_type = self.vs.getSeqType(param_type)
		else:
			seq_type = param_type
		
		if seq_type == "int":
			return "std_msgs::Int32"
		elif seq_type == "long":
			return "std_msgs::Int32"
		elif seq_type == "float":
			return "std_msgs::Float32"
		elif seq_type == "double":
			return "std_msgs::Float64"
		elif seq_type == "string":
			return "std_msgs::String"

	def getRosMsgType(self, param_type, param_kind):
		ret = self.getRosMsgSeqType(param_type, param_kind)
		if param_kind == idltype.tk_sequence:
			if param_type == 'string':
				raise Exception("ERROR: String arrays are not supported by ROS.")
			return ret + "MultiArray"
		return ret

	def getRosMsgIterType(self, param_type, param_kind):
		if param_kind != idltype.tk_sequence:
			return
		return "std::vector< " + self.vs.getSeqType(param_type) + ">::const_iterator"

