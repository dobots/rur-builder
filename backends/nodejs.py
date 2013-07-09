# -*- python -*-
#                           Package   : omniidl
# nodejs.py                 Created on: 2013/04/01
#                           Author    : Bart van Vliet
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
#   NodeJS back-end for network abstractions 

"""NodeJS IDL compiler back-end."""

# http://bespin.cz/~ondras/html/ v8 api reference
# Example: https://github.com/kkaefer/node-cpp-modules
# Example: https://bravenewmethod.wordpress.com/2011/03/30/callbacks-from-threaded-node-js-c-extension/
# More info: http://nikhilm.github.io/uvbook/threads.html
# node::ObjectWrap::Ref - Keep your C++ objects around even if they aren't referenced.
# v8::Signature - Ensure that "this" always refers to your wrapped C++ object.
# TODO: add TryCatch to catch errors.
# TODO: add ThrowException to show errors
# TODO: add char support
# TODO: use ->IntegerValue() instead of ->NumberValue() for integers

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur




class NodeJS:

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
		print "#include <deque>"
		print "#include <node.h>"
		print "#include <pthread.h>"

	def writeIncludesImpl(self):
		print "#include <" + self.vs.classname + "Ext.h>"

	def writeUsingNamespace(self):
		self.st.out("using namespace v8;")

# Class
	def writeClassStart(self):
		self.st.out("class " + self.vs.classname + " : public node::ObjectWrap {")

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
		self.st.out("pthread_t moduleThread;")
		self.st.out("bool DestroyFlag;")
		self.st.out("pthread_mutex_t destroyMutex;")
		self.st.out("")

	def writePrivateFuncsDecl(self):
		self.writeNodeConstructor()
		self.writeNodeDestructor()

	def writePrivatePortVarsDecl(self, p):
		self.writeBufAllocation(p)

	def writePrivatePortFuncsDecl(self, p):
		self.writeNodePort(p)

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
		self.writeRun()
		self.writeNodeRegister()

	def writePublicPortVarsDecl(self, p):
		pass

	def writePublicPortFuncsDecl(self, p):
		self.writePort(p)

# Constructor implementation
	def writeConstructorImplStart(self):
		self.st.out(self.classname + "::" + self.classname + "():")

	def writeConstructorImplInit(self):
		pass

	def writeConstructorImplPortInit(self, p):
		pass

	def writeConstructorImpl(self):
		self.st.out("DestroyFlag = false;")

	def writeConstructorImplPort(self, p):
		self.writePortBufInitiation(p)

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
		self.writeNodeRun()
		self.writeRunImpl()
		self.writeNodeConstructorImpl()
		self.writeNodeDestructorImpl()
		self.writeNodeRegisterImpl()

	def writePortFuncsImpl(self, p):
		self.writeNodePortImpl(p)
		self.writePortImpl(p)



##########################################################################################
##################################### Write function #####################################
##########################################################################################

	def writeBufAllocation(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments= self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			#self.st.out(param_type + "_buf readBuf" + port_name + ";")
			self.st.out("std::deque<" + param_type + "> readBuf" + port_name + ";")
			self.st.out(param_type + " readVal" + port_name  + ";")
			self.st.out("pthread_mutex_t readMutex" + port_name + ";")
			self.st.out("")
		if port_direction == rur.Direction.OUT:
			#self.st.out(param_type + "_node writeBuf" + port_name + ";")
			self.st.out("std::deque<" + param_type + "> writeBuf" + port_name + ";")
			self.st.out("v8::Persistent<v8::Function> nodeCallBack" + port_name + ";")
			self.st.out("uv_async_t async" + port_name + ";")
			self.st.out("pthread_mutex_t writeMutex" + port_name + ";")
			self.st.out("")

	def writeNodeConstructor(self):
		self.st.out("static v8::Handle<v8::Value> NodeNew(const v8::Arguments& args);")
		self.st.out("")

	def writeNodeDestructor(self):
		self.st.out("static v8::Handle<v8::Value> NodeDestroy(const v8::Arguments& args);")
		self.st.out("")
		self.st.out("bool Destroy();")
		self.st.out("")

	def writeNodeRegister(self):
		self.st.out("// Function template for NodeJS, do not use in your own code")
		self.st.out("static void NodeRegister(v8::Handle<v8::Object> exports);")
		self.st.out("")

	def writeRun(self):
		self.st.out("void Run();")
		self.st.out("")

	def writePort(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
#		if port_direction == rur.Direction.IN:
#			self.vs.writePortFunctionSignature(p)
#		if port_direction == rur.Direction.OUT:
#			self.vs.writePortFunctionSignature(p)
		self.vs.writePortFunctionSignature(p)

	def writePortBufInitiation(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("readBuf" + port_name + " = " + "std::deque<" + param_type + ">(0);")
			self.st.out("readVal" + port_name + " = " + param_type + "(0);")
		if port_direction == rur.Direction.OUT:
			self.st.out("writeBuf" + port_name + " = " + "std::deque<" + param_type + ">(0);")

	def writeNodePort(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			# Port IN, so in javascript you write to it
			self.st.out("// Function to be used in NodeJS, not in your C++ code")
			self.st.out("static v8::Handle<v8::Value> NodeWrite" + port_name + "(const v8::Arguments& args);")
			self.st.out("")
			
		if port_direction == rur.Direction.OUT: 
			# Port OUT, so in javascript you read from it
			self.st.out("// Function to be used in NodeJS, not in your C++ code")
			self.st.out("static v8::Handle<v8::Value> NodeRegRead" + port_name + "(const v8::Arguments& args);")
			self.st.out("")
			
			# The callback function that calls the javascript callback
			self.st.out("// Function to be used internally, not in your C++ code")
			self.st.out("static void CallBack" + port_name + "(uv_async_t *handle, int status);")
			self.st.out("")


	def writeNodeRun(self):
		self.st.out("static void* RunModule(void* object) {")
		self.st.inc_indent()
		self.st.out("static_cast<" + self.vs.classname + "Ext*>(object)->Run();")
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writeRunImpl(self):
		self.st.out("void " + self.vs.classname + "::Run() {")
		self.st.inc_indent()
		self.st.out("while (true) {")
		self.st.inc_indent()
		self.st.out("Tick();")
		self.vs.writeFunctionEnd()
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writeNodeConstructorImpl(self):
		self.st.out("v8::Handle<v8::Value> " + self.vs.classname + "::NodeNew(const v8::Arguments& args) {")
		self.st.inc_indent()
		self.st.out("v8::HandleScope scope;")
		self.st.out("if ((args.Length() < 1) || (!args[0]->IsString())) {")
		self.st.inc_indent()
		self.st.out("return v8::ThrowException(v8::String::New(\"Invalid argument, must provide a string.\"));")
		self.vs.writeFunctionEnd()
		self.st.out("" + self.vs.classname + "Ext* obj = new " + self.vs.classname + "Ext();")
		self.st.out("obj->Wrap(args.This());")
		self.st.out("")
		self.st.out("std::string name = (std::string)*v8::String::AsciiValue(args[0]);")
		self.st.out("obj->Init(name);")
		self.st.out("")
		self.st.out("pthread_mutex_init(&(obj->destroyMutex), NULL);")
		self.st.out("")
		self.st.out("// Init ports")
		for p in self.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
			self.writePortInit(p)
		self.st.out("// Start the module loop")
		self.st.out("pthread_create(&(obj->moduleThread), 0, RunModule, obj);")
		self.st.out("")
		self.st.out("// Make this object persistent")
		self.st.out("obj->Ref();")
		self.st.out("")
		self.st.out("return args.This();")
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writeNodeDestructorImpl(self):
		self.st.out("v8::Handle<v8::Value> " + self.vs.classname + "::NodeDestroy(const v8::Arguments& args) {")
		self.st.inc_indent()
		self.st.out("v8::HandleScope scope;")
		self.st.out("" + self.vs.classname + "Ext* obj = ObjectWrap::Unwrap<" + self.vs.classname + "Ext>(args.This());")
		#self.st.out("bool destroy = obj->Destroy();")
		self.st.out("return scope.Close(v8::Boolean::New(obj->Destroy()));")
		self.vs.writeFunctionEnd()
		self.st.out("")
		self.st.out("bool " + self.vs.classname + "::Destroy() {")
		self.st.inc_indent()
		self.st.out("bool canDestroy = true;")
		for p in self.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
			if port_direction == rur.Direction.OUT:
				self.st.out("if (canDestroy) {")
				self.st.inc_indent()
				self.st.out("pthread_mutex_lock(&writeMutex" + port_name + ");")
				self.st.out("if (!writeBuf" + port_name + ".empty())")
				self.st.inc_indent()
				self.st.out("canDestroy = false;")
				self.st.dec_indent()
				self.st.out("pthread_mutex_unlock(&writeMutex" + port_name + ");")
				self.vs.writeFunctionEnd()
		self.st.out("if (canDestroy) {")
		self.st.inc_indent()
		self.st.out("pthread_cancel(moduleThread);")
		self.st.out("Unref();")
		self.st.out("return true;")
		self.vs.writeFunctionEnd()
		self.st.out("else {")
		self.st.inc_indent()
		self.st.out("pthread_mutex_lock(&destroyMutex);")
		self.st.out("DestroyFlag = true;")
		self.st.out("pthread_mutex_unlock(&destroyMutex);")
		self.st.out("return true; // return true anyway?")
		self.vs.writeFunctionEnd()
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writeNodeRegisterImpl(self):
		self.st.out("void " + self.vs.classname + "::NodeRegister(v8::Handle<v8::Object> exports) {")
		self.st.inc_indent()
		self.st.out("v8::Local<v8::FunctionTemplate> tpl = v8::FunctionTemplate::New(NodeNew);")
		self.st.out("tpl->SetClassName(v8::String::NewSymbol(\"" + self.vs.classname + "\"));")
		self.st.out("tpl->InstanceTemplate()->SetInternalFieldCount(1);")
		self.st.out("")
		self.st.out("// Prototypes")
		self.st.out("tpl->PrototypeTemplate()->Set(v8::String::NewSymbol(\"Destroy\"), v8::FunctionTemplate::New(NodeDestroy)->GetFunction());")
		for p in self.portList:
			port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
			if port_direction == rur.Direction.IN:
				self.st.out("tpl->PrototypeTemplate()->Set(v8::String::NewSymbol(\"Write" + port_name + "\"), v8::FunctionTemplate::New(NodeWrite" + port_name + ")->GetFunction());")
			if port_direction == rur.Direction.OUT:
				self.st.out("tpl->PrototypeTemplate()->Set(v8::String::NewSymbol(\"RegRead" + port_name + "\"), v8::FunctionTemplate::New(NodeRegRead" + port_name + ")->GetFunction());")
		self.st.out("")
		self.st.out("v8::Persistent<v8::Function> constructor = v8::Persistent<v8::Function>::New(tpl->GetFunction());")
		self.st.out("exports->Set(v8::String::NewSymbol(\"" + self.vs.classname + "\"), constructor);")
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writePortInit(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("pthread_mutex_init(&(obj->readMutex" + port_name + "), NULL);")
			self.st.out("")
		if port_direction == rur.Direction.OUT:
			self.st.out("pthread_mutex_init(&(obj->writeMutex" + port_name + "), NULL);")
			self.st.out("uv_async_init(uv_default_loop() , &(obj->async" + port_name + "), &(obj->CallBack" + port_name + "));")
			self.st.out("")

	def writeNodePortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			# Port IN, so in javascript you write to it
			self.st.out("v8::Handle<v8::Value> " + self.vs.classname + "::NodeWrite" + port_name + "(const v8::Arguments& args) {")
			self.st.inc_indent()
			self.st.out("v8::HandleScope scope;")
			self.st.out(self.vs.classname + "Ext* obj = ObjectWrap::Unwrap<" + self.vs.classname + "Ext>(args.This());")
			#self.st.out("//type=" + param_type + " kind=" + str(param_kind))
			if param_kind == idltype.tk_sequence:
				self.st.out("if (args.Length() < 1 || !args[0]->IsArray())")
				self.st.inc_indent()
				self.st.out("return scope.Close(v8::Boolean::New(false)); // Could also throw an exception")
				self.st.dec_indent()
				self.st.out("v8::Handle<v8::Array> arr = v8::Handle<v8::Array>::Cast(args[0]);")
				self.st.out("unsigned int len = arr->Length();")
				#self.st.out(param_type + " readVec(0);")
				self.st.out("pthread_mutex_lock(&(obj->readMutex" + port_name + "));")
				#self.st.out("obj->readBuf" + port_name + ".push_back(readVec);")
				self.st.out("obj->readBuf" + port_name + ".resize(obj->readBuf" + port_name + ".size()+1);")
				self.st.out(param_type + "& readVec = obj->readBuf" + port_name + ".back();")
				self.st.out("for (unsigned int i=0; i<len; ++i)")
				self.st.inc_indent()
				#self.st.out("obj->readBuf" + port_name + ".back().push_back(arr->Get(v8::Integer::New(i))->NumberValue());")
				self.st.out("readVec.push_back(arr->Get(v8::Integer::New(i))->NumberValue());")
				self.st.dec_indent()
			else:
				self.st.out("if (args.Length() < 1)")
				self.st.inc_indent()
				self.st.out("return scope.Close(v8::Boolean::New(false)); // Could also throw an exception")
				self.st.dec_indent()
				self.st.out("pthread_mutex_lock(&(obj->readMutex" + port_name + "));")
				self.st.out("obj->readBuf" + port_name + ".push_back(args[0]->NumberValue());")
			self.st.out("pthread_mutex_unlock(&(obj->readMutex" + port_name + "));")
			self.st.out("return scope.Close(v8::Boolean::New(true));")
			self.vs.writeFunctionEnd()
			self.st.out("")
			
		if port_direction == rur.Direction.OUT: 
			# Port OUT, so in javascript you read from it
			self.st.out("v8::Handle<v8::Value> " + self.vs.classname + "::NodeRegRead" + port_name + "(const v8::Arguments& args) {")
			self.st.inc_indent()
			self.st.out("v8::HandleScope scope;")
			self.st.out(self.vs.classname + "Ext* obj = ObjectWrap::Unwrap<" + self.vs.classname + "Ext>(args.This());")
			self.st.out("if (args.Length() < 1 || !args[0]->IsFunction())")
			self.st.inc_indent()
			self.st.out("return scope.Close(v8::Boolean::New(false)); // Could also throw an exception")
			self.st.dec_indent()
			self.st.out("v8::Local<v8::Function> callback = v8::Local<v8::Function>::Cast(args[0]);")
			self.st.out("obj->nodeCallBack" + port_name + " = v8::Persistent<v8::Function>::New(callback);")
			self.st.out("return scope.Close(v8::Boolean::New(true));")
			self.vs.writeFunctionEnd()
			self.st.out("")
			
			# The callback function that calls the javascript callback
			self.st.out("void " + self.vs.classname + "::CallBack" + port_name + "(uv_async_t *handle, int status) {")
			self.st.inc_indent()
			self.st.out("v8::HandleScope scope;")
			self.st.out(self.vs.classname + "Ext* obj = (" + self.vs.classname + "Ext*)(handle->data);")
			self.st.out("const unsigned argc = 1;")
			self.st.out("while (true) {") # begin while loop
			self.st.inc_indent()
			self.st.out("pthread_mutex_lock(&(obj->writeMutex" + port_name + "));")
			self.st.out("if (obj->writeBuf" + port_name + ".empty()) {")
			self.st.inc_indent()
			self.st.out("pthread_mutex_unlock(&(obj->writeMutex" + port_name + ")); // Don't forget to unlock!")
			self.st.out("break;")
			self.vs.writeFunctionEnd()
			if param_kind == idltype.tk_sequence:
				self.st.out("unsigned int len;")
				self.st.out("v8::Handle<v8::Array> arr;")
				self.st.out("if (!obj->nodeCallBack" + port_name + ".IsEmpty()) {")
				self.st.inc_indent()
				self.st.out("len = obj->writeBuf" + port_name + ".front().size();")
				self.st.out("arr = v8::Array::New(len);")
				self.st.out(param_type + "& vec = obj->writeBuf" + port_name + ".front();")
				self.st.out("for (unsigned int i=0; i<len; ++i)")
				self.st.inc_indent()
				self.st.out("arr->Set(v8::Integer::New(i), v8::Number::New(vec[i]));")
				self.st.dec_indent()
				self.vs.writeFunctionEnd()
				self.st.out("obj->writeBuf" + port_name + ".pop_front();")
				self.st.out("pthread_mutex_unlock(&(obj->writeMutex" + port_name + "));")
				self.st.out("if (!obj->nodeCallBack" + port_name + ".IsEmpty()) {")
				self.st.inc_indent()
				self.st.out("v8::Local<v8::Value> argv[argc] = { v8::Local<v8::Value>::New(arr) };")
				self.st.out("obj->nodeCallBack" + port_name + "->Call(v8::Context::GetCurrent()->Global(), argc, argv);")
				self.vs.writeFunctionEnd()
			else:
				self.st.out("v8::Local<v8::Value> argv[argc] = { v8::Local<v8::Value>::New(v8::Number::New(obj->writeBuf" + port_name + ".front())) };")
				self.st.out("obj->writeBuf" + port_name + ".pop_front();")
				self.st.out("pthread_mutex_unlock(&(obj->writeMutex" + port_name + "));")
				self.st.out("if (!obj->nodeCallBack" + port_name + ".IsEmpty())")
				self.st.inc_indent()
				self.st.out("obj->nodeCallBack" + port_name + "->Call(v8::Context::GetCurrent()->Global(), argc, argv);")
				self.st.dec_indent()
			self.vs.writeFunctionEnd() # end of while loop
			self.st.out("pthread_mutex_lock(&(obj->destroyMutex));")
			self.st.out("bool destroy = obj->DestroyFlag;")
			self.st.out("pthread_mutex_unlock(&(obj->destroyMutex));")
			self.st.out("if (destroy)")
			self.st.inc_indent()
			self.st.out("obj->Destroy();")
			self.st.dec_indent()
			self.vs.writeFunctionEnd()
			self.st.out("")

	def writePortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.IN)
			self.st.out("pthread_mutex_lock(&destroyMutex);")
			self.st.out("bool destroy = DestroyFlag;")
			self.st.out("pthread_mutex_unlock(&destroyMutex);")
			self.st.out("if (destroy)")
			self.st.inc_indent()
			if (param_kind == idltype.tk_sequence):
				self.st.out("return &readVal" + port_name + ";")
				self.st.dec_indent()
				self.st.out("pthread_mutex_lock(&readMutex" + port_name + ");")
				self.st.out("if (!readBuf" + port_name + ".empty()) {")
				self.st.inc_indent()
				self.st.out("readVal" + port_name + ".swap(readBuf" + port_name + ".front());")
				self.st.out("readBuf" + port_name + ".pop_front();")
				self.vs.writeFunctionEnd()
				self.st.out("pthread_mutex_unlock(&readMutex" + port_name + ");")
				self.st.out("return &readVal" + port_name + ";")
			else:
				self.st.out("return NULL;")
				self.st.dec_indent()
				self.st.out("pthread_mutex_lock(&readMutex" + port_name + ");")
				self.st.out("if (readBuf" + port_name + ".empty()) {")
				self.st.inc_indent()
				self.st.out("pthread_mutex_unlock(&readMutex" + port_name + "); // Don't forget to unlock!")
				self.st.out("return NULL;")
				self.vs.writeFunctionEnd()
				self.st.out("readVal" + port_name + " = readBuf" + port_name + ".front();")
				self.st.out("readBuf" + port_name + ".pop_front();")
				self.st.out("pthread_mutex_unlock(&readMutex" + port_name + ");")
				self.st.out("return &readVal" + port_name + ";")
			self.vs.writeFunctionEnd()
			self.st.out("")
			
		if port_direction == rur.Direction.OUT:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.OUT)
			self.st.out("pthread_mutex_lock(&destroyMutex);")
			self.st.out("bool destroy = DestroyFlag;")
			self.st.out("pthread_mutex_unlock(&destroyMutex);")
			self.st.out("if (destroy)")
			self.st.inc_indent()
			self.st.out("return false;")
			self.st.dec_indent()
			self.st.out("pthread_mutex_lock(&writeMutex" + port_name + ");")
			self.st.out("writeBuf" + port_name + ".push_back(" + param_name + ");")
			self.st.out("pthread_mutex_unlock(&writeMutex" + port_name + ");")
			self.st.out("async" + port_name + ".data = (void*) this;")
			self.st.out("uv_async_send(&async" + port_name + ");")
			self.st.out("return true;")
			self.vs.writeFunctionEnd()
			self.st.out("")
