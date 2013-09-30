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
import sys, string, os

sys.path.append('./helper')
from helper import rur
from rur_main_visitor import MainVisitor
from rur_main import Main

#===============================================================================
#===============================================================================
# # The Relay IDL protocol
#===============================================================================
#	A relay is defined as input and output pairs with the SAME variable_name	
#
# 	An input is defined as:									variable_name__In
# 		optionally, there can be multiple input channels:
# 			variable_name__In_0, variable_name__In_1 etc.
# 	
# 	Each Input need and output. This output is defined as: 	variable_name__Out
# 		optionally, there can be multiple output channels:
# 			variable_name__Out_0, variable_name__Out_1 etc.
# 
# 	Typecasting will be done if required.
#
#===============================================================================
class Main_relay (Main):
	
	def __init__(self, st, tree, args):
		self.basename = self.getBasenameFromPath(args[1])
		self.aim_path = self.getFilenameFromTemplatePath(args[2])
		print "//",args
		Main.__init__(self,st,tree,args)
	
	def writeAll(self):
 		file_handle = open(self.aim_path, 'r')
 		content = file_handle.read()
 		file_handle.close()
 		
 		# replace the template titles with the project specific ones
 		content = content.replace("TemplateModuleExt",self.basename)
 		
 		# insert dependencies
 		content = content.replace("using namespace rur;","#include <unistd.h>\n#include <iostream>\nusing namespace rur;")
 		
 		
		template_parts = content.split("Ext::Tick() {")
		
		print template_parts[0] + "Ext::Tick() {"
		self.st.inc_indent()
 		for i,port in enumerate(self.vs.portList):
 			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(port)
 			
 			# if port in an input port
 			if port_direction == 0:
 				# if the port adheres to the relay protocol
 				if self.checkForInputIdentifier(port_name):
	 				outputs = self.checkForOutput(port_name)
	 				print "// outputs : ", outputs
	 				#if this input has output(s)
	 				if len(outputs) != 0:
			 			self.st.out(param_type + "* relay_var" + str(i) + " = read" + port_name + "(false);")
			 			if param_kind == idltype.tk_sequence:
				 			# check if new message is available
				 			self.st.out("if (relay_var" + str(i) + " != NULL && !relay_var" + str(i) + "->empty()) {")
				 			self.st.inc_indent()
				 			self.handleOutputs(port_name,param_type,param_kind,outputs,'relay_var' + str(i))
				 			self.st.dec_indent()
				 		else:
				 			# if the data is not an array type, the handling is a little different.
				 			self.st.out("if (relay_var" + str(i) + " != NULL) {")
				 			self.st.inc_indent()
				 			self.handleOutputs(port_name,param_type,param_kind,outputs,'relay_var' + str(i))
				 			self.st.dec_indent()
			 			self.st.out("}\n")
	 			
 		self.st.dec_indent()
 		
#  		self.st.out("usleep(sleepTime);")
 		
  		print template_parts[1]
  		
  		# print full port information at the bottom
  		print "/*"
  		for port in self.vs.portList:
 			print self.vs.getPortConfiguration(port)
  		print "*/"
  		
  	def handleOutputs(self,input_name,input_param_type,input_param_kind,outputs,relay_var_name):
  		for output_name,output_param_type,output_param_kind in outputs:
  			# if both are sequences or single values
  			if output_param_kind == input_param_kind == idltype.tk_sequence:
  				self.st.out("write"+output_name+"(*"+relay_var_name+");")
  			elif input_param_kind == idltype.tk_sequence:
	  			self.st.out("for (int i=0; i < " + relay_var_name + "->size(); i++)")
	  			self.st.inc_indent()
	  			if output_param_type == input_param_type:
	  		 		self.st.out("write"+output_name+"("+relay_var_name+"->at(i));")
  		 		else:
	  			  self.st.out("write"+output_name+"("+output_param_type+"("+relay_var_name+"->at(i)));")
	  			self.st.dec_indent()
	  		else:
	  			if output_param_type == input_param_type:
	  		 		self.st.out("write"+output_name+"(*"+relay_var_name+");")
  		 		else:
  		 		 	self.st.out("write"+output_name+"("+output_param_type+"(*"+relay_var_name+"));")
#===============================================================================
# 	  		
# # loop over entrees and output.. change this to send later
# 			self.st.out("for (int i=0; i < relay_var" + str(i) + "->size(); i++) {")
# 			self.st.inc_indent()
# 			self.st.out("std::cout << relay_var" + str(i) + "->at(i) << std::endl;")
# 				
# 			# close the for and if
# 			self.st.dec_indent()
# 		self.st.out("}")
#===============================================================================
		
		
  	def checkForInputIdentifier(self,input_portname):
		name_array = str(input_portname).split("__")
		# if there is an in or output definition
		if len(name_array) > 1:
			direction_array = name_array[-1].split("_")
			# if this is an output port
			if direction_array[0] == "In":
				return True
			else:
				return False
		else:
			return False
		
	def checkForOutput(self,input_portname):
		outputs = []
		input_base_portname = str(input_portname).split("__")[0]
		for i,port in enumerate(self.vs.portList):
 			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(port)
 			name_array = str(port_name).split("__")
 			#if the port is in the input set
 			if name_array[0] == input_base_portname:
	 			# if there is an in or output definition
	 			if len(name_array) > 1:
	 				direction_array = name_array[-1].split("_")
	 				# if this is an output port
	 				if direction_array[0] == "Out":
		 				outputs.append([port_name, param_type, param_kind])
		return outputs
	
	def getFilenameFromTemplatePath(self,path):
		path_array = path.split("/")
		path_array.pop()
		path_array.append("templates")
		path_array.append("src")
		path_array.append("TemplateModuleExt.cpp")
		return "/".join(path_array)
	
	def getBasenameFromPath(self,path):
		path_array = path.split("/")
		return path_array[-1].replace(".cpp","")
		
		
# Initialize this parser
def run(tree, args):
	st = output.Stream(sys.stdout, 2)
	main = Main_relay(st, tree, args)
	
	# And then write everything to the to-be-generated header file
	main.writeAll()

