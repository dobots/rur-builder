# -*- python -*-
#                           Package   : omniidl
# zeromq.py                 Created on: 2013/02/24
#                           Author    : Anne C. van Rossum
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
#   ZeroMQ back-end for network abstractions 

"""ZeroMQ IDL compiler back-end."""

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur

# TODO: finish coversion of self.writePort
# TODO: Close() should this be a default function?
# TODO: port->writeStrict() and port->setStrict()

class ZeroMQ:

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
		print "#include <string>"
		print "#include <sstream>"
		print "#include <unistd.h>" # for getpid()
		print "// zeromq specific headers"
		print "#include <zmq.hpp>"
		print "#include <json_spirit_reader.h>"
		print

	def writeIncludesImpl(self):
		pass

	def writeUsingNamespace(self):
		pass

# Before class
	def writeBeforeClassVarsDecl(self):
		self.writeStructs()

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
		self.st.out("// the socket over which is communicated with the name server")
		self.st.out("zmq::socket_t *ns_socket;")
		self.st.out("// standard control socket over which commands arrive to connect to some port for example")
		self.st.out("zmq::socket_t *cmd_socket;")
		self.st.out("// standard control socket over which commands arrive to connect to some port for example")
		self.st.out("std::vector<zmq_socket_ext*> zmq_sockets;")

	def writePrivateFuncsDecl(self):
		pass

	def writePrivatePortVarsDecl(self, p):
		self.writePortDeclaration(p)

	def writePrivatePortFuncsDecl(self, p):
		pass

# Protected
	def writeProtectedVarsDecl(self):
		self.st.out("// the standard zeromq context object")
		self.st.out("zmq::context_t *context;")
		self.st.out("// some default debug parameter")
		self.st.out("char debug;")

	def writeProtectedFuncsDecl(self):
		self.writeHelperFunctions()

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

# Constructor implementation
	def writeConstructorImplStart(self):
		self.st.out(self.classname + "::" + self.classname + "():")

	def writeConstructorImplInit(self):
		self.writeCtorInit()

	def writeConstructorImplPortInit(self, p):
		pass

	def writeConstructorImpl(self):
		self.writeCtor()

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
		self.writeInit()

	def writeInitImplPort(self, p):
		self.writePortInit(p)

# Implementation
	def writeFuncsImpl(self):
		self.writeHelperFunctionsImpl()

	def writePortFuncsImpl(self, p):
		self.writePortImpl(p)



##########################################################################################
##################################### Write function #####################################
##########################################################################################


	def writeFileComment(self):
		comment_text = '''
/***********************************************************************************************************************
 * Remarks
 **********************************************************************************************************************/

// zmq_ctx_new used to be zmq_init which is by default installed in Ubuntu with apt-get install libzmq-dev
// so you'll need to deinstall that... and install zmq from source
// see: http://zguide.zeromq.org/cpp:hwserver and http://zguide.zeromq.org/cpp:hwclient
// see: http://zguide.zeromq.org/c:hwserver

'''
		print comment_text

	def writeStructs(self):
		self.st.out('/**')
		self.st.out(' * Port name service record. This is like a domain name service record, but instead of containing an IP address and an')
		self.st.out(' * URI, it comes with a "name" that can be resolved as a "host", "port", and "pid". The name is something like "/write",')
		self.st.out(' * the host something like "127.0.0.1" or "dev.almende.com" (that is resolvable by dns), "port" is a TCP/UDP port, and')
		self.st.out(' * "pid" is the process identifier.')
		self.st.out(' */')
		self.st.out('typedef struct psn_record_t {')
		self.st.inc_indent()
		self.st.out('std::string name;')
		self.st.out('std::string host;')
		self.st.out('std::string port;')
		self.st.out('std::string pid;')
		self.st.dec_indent()
		self.st.out('} pns_record;')
		self.st.out("")
		self.st.out('// Following structure makes it easier to store state information per socket')
		self.st.out('typedef struct zmq_socket_ext_t {')
		self.st.inc_indent()
		self.st.out('zmq::socket_t *sock;')
		self.st.out('std::string name;')
		self.st.out('bool ready;')
		self.st.dec_indent()
		self.st.out('} zmq_socket_ext;')
		self.st.out("")

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
#			portValue = self.getPortValue(p)
#			portValueName = self.getPortValueName(node,p)
#			self.st.out("// private storage for " + portValueName)
#			self.st.out(portValue + " " + portValueName + ";")
			self.st.out(param_type + " " + port + "Value;")

#			self.st.out("// the port " + portname + " itself") 
			self.st.out("zmq_socket_ext " + port + "In;")
		if port_direction == rur.Direction.OUT:
			self.st.out("zmq_socket_ext " + port + "Out;")
#		self.st.out("zmq_socket_ext " + port + ";")

	def writeCtorInit(self):
		self.st.out("ns_socket(NULL),")
		self.st.out("cmd_socket(NULL),")
		self.st.out("debug(0),")

	def writeCtor(self):
		self.st.out("context = new zmq::context_t(1);")

	# In the constructor we allocate the port, most often we will need a new BufferedPort with a 
	# Bottle as parameter. In case of a sequence we need to allocate a corresponding vector
	def writePortAllocation(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out(port + "In.sock = new zmq::socket_t(*context, ZMQ_REP);")
			self.st.out(port + "In.ready = true;")
			self.st.out("zmq_sockets.push_back(&" + port + "In);")
		if port_direction == rur.Direction.OUT:
			self.st.out(port + "Out.sock = new zmq::socket_t(*context, ZMQ_REQ);")
			self.st.out(port + "Out.ready = true;")
			self.st.out("zmq_sockets.push_back(&" + port + "Out);")

	def writePortDestruction(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("delete " + port + "In.sock;")
		if port_direction == rur.Direction.OUT:
			self.st.out("delete " + port + "Out.sock;")

	def writeInit(self):
		self.st.out("std::cout << \"Connecting to name server...\" << std::endl;")
		self.st.out("ns_socket = new zmq::socket_t(*context, ZMQ_REQ);")
		self.st.out("try {")
		self.st.inc_indent()
		self.st.out("ns_socket->connect(\"tcp://127.0.0.1:10101\"); // port to connect to, REQ/REP")
		self.st.dec_indent()
		self.st.out("} catch (zmq::error_t & e) {")
		self.st.inc_indent()
		self.st.out("std::cerr << \"Error: Could not connect to the name server: = \" << e.what() << std::endl;")
		self.vs.writeFunctionEnd()
		self.st.out("cmd_socket = new zmq::socket_t(*context, ZMQ_REP);")
		self.st.out("")
		self.st.out("std::string zmqPortName;")
		self.st.out("std::stringstream zmqSs;")
		self.st.out("{")
		self.st.inc_indent()
		self.st.out("pns_record record;")
		self.st.out("zmqSs.clear(); zmqSs.str(\"\");" )
		self.st.out("zmqSs << getpid();" )
		self.st.out("record.name = \"/resolve/\" + zmqSs.str() + \"/control\";")
		self.st.out("record.pid = zmqSs.str();" )
		self.st.out("Resolve(record);" )
		self.st.out("zmqSs.clear(); zmqSs.str(\"\");" )
		self.st.out("zmqSs << \"tcp://\" << record.host << \":\" << record.port; ")
		self.st.out("std::string zmqPortName = zmqSs.str(); ")
		self.st.out("std::cout << \"Bind to socket \" << zmqPortName << std::endl; ")
		self.st.out("cmd_socket->bind(zmqPortName.c_str());")
		self.vs.writeFunctionEnd()
		self.st.out("")	

	def writePortInit(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		self.st.out("{")
		self.st.inc_indent()
		if port_direction == rur.Direction.IN:
			self.st.out("// incoming port, function as client")
			portname = port + "In"
		if port_direction == rur.Direction.OUT:
			self.st.out("// outgoing port, function as server")
			portname = port + "Out"
	#	self.st.out("zmqSs.str(\"\"); zmqSs.clear();")
	#	self.st.out('zmqSs << "/' + self.classname.lower() + '" << cliParam->module_id << "/' + port_name + '";')
	#	self.st.out(port + ".name = zmqSs.str();")
		self.st.out('zmqPortName = "/' + self.vs.classname.lower() + '" + cliParam->module_id + "/' + port.lower() + '";')
		self.st.out(portname + ".name = zmqPortName;")
	#	self.st.out("std::string resolve = \"/resolve\" + portName.str();" )
		self.st.out('zmqPortName = "/resolve" + ' + portname + '.name;')
		self.st.out("pns_record record;")
		self.st.out("record.name = zmqPortName;")
		self.st.out("zmqSs.clear(); zmqSs.str(\"\");")
		self.st.out("zmqSs << getpid();")
		self.st.out("record.pid = zmqSs.str();")
		self.st.out("Resolve(record);")
		if port_direction == rur.Direction.IN:
			self.st.out("zmqSs.str(\"\");")
			self.st.out('zmqSs << "tcp://" << record.host << ":" << record.port;')
			self.st.out("zmqPortName = zmqSs.str();")
			self.st.out("std::cout << \"Bind to socket \" << zmqPortName << std::endl;")
			self.st.out(port + "In.sock->bind(zmqPortName.c_str());")
		self.vs.writeFunctionEnd()

	def writePortClose(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out(port + "In.sock->close();")
		if port_direction == rur.Direction.OUT:
			self.st.out(port + "Out.sock->close();")


	# The ports themselves will become again functions, like readInput() or writeOutput()
	# The result of this function will be a list of such functions
	# All these functions will be "protected" and can be accessed only by the class or its parent.
	def writePort(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("/**")
			self.st.out(" * The \"read" + port_name + "\" function receives stuff over a zeromq REP socket. It works as a client. It is better not")
			self.st.out(" * to run it in blocking mode, because this would make it impossible to receive message on other ports (under which ")
			self.st.out(" * the /pid/control port). The function returns NULL if there is no new item available.")
			self.st.out(" */")
		if port_direction == rur.Direction.OUT:
			self.st.out("/**")
			self.st.out(" * The \"write" + port_name + "\" function sends stuff over a zeromq REQ socket. It works as a server. It cannot be blocking because this")
			self.st.out(" * would make it impossible to receive message on other ports (under which the /pid/control port). It could have been")
			self.st.out(" * blocking if it is known if it is connected to a REP port (but the connected() function is apparently not meant for")
			self.st.out(" * that).")
			self.st.out(" */")
		self.vs.writePortFunctionSignature(p)
		# TODO: writePortFunctionSignature writes both in and out functions
		# split up like writePortFunctionSignatureImpl?

	def writePortImpl(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, pragmas, comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.IN)
			self.st.out("// For now only int return values are supported")
			self.st.out("int reply_size = -1;")
			self.st.out("char *reply = GetReply(" + port + "In.sock, " + port + "In.ready, blocking, reply_size);")
			self.st.out("if (!" + port + "In.ready || !reply) return NULL;")
			self.st.out("SendAck(" + port + "In.sock, " + port + "In.ready);")
#			self.st.out("std::cout << \"reply_size = \" << reply_size << \" reply=\" << std::string(reply) << std::endl;")
#			self.st.out('for (int i=0; i<reply_size; ++i) std::cout << +reply[i] << " ";')
#			self.st.out("std::cout << std::endl;")
			self.st.out("if (reply_size < 2) std::cerr << \"Error: Reply is not large enough to store a value!\" << std::endl;")
			self.st.out("std::stringstream ss; ss.clear(); ss.str(\"\");")
			self.st.out("ss << std::string(reply);")
			if param_kind == idltype.tk_sequence:
				self.st.out(self.vs.getSeqType(param_type) + " itemVal;")
				self.st.out("std::stringstream ssItem;")
				self.st.out(port + "Value.clear();")
#				self.st.out("assert(false);  // todo: cast char* array to vector")
#				self.st.out("istringstream iss(ss);")
#				self.st.out("vector<string> tokens;")
#				self.st.out("copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));")
				self.st.out("std::string item;")
				self.st.out("char delim(' ');")
#				self.st.out('std::cout << "received: " << ss.str() << std::endl;')
				self.st.out("while (std::getline(ss, item, delim)) {")
				self.st.inc_indent()
				self.st.out("ssItem.clear(); ssItem.str(\"\");")
				self.st.out("ssItem << item;")
				self.st.out("ssItem >> itemVal;")
				self.st.out(port + "Value.push_back(itemVal);")
				self.vs.writeFunctionEnd()
			else:
				self.st.out("ss >> " + port + "Value;")
#				self.st.out(portname + "Value = (reply[0]) + (reply[1] << 8); // little-endianness")
#				self.st.out(" std::cout << \"Values\" << (reply[0]) << \" and \" << (reply[1]) << std::endl; ")
			self.st.out("delete [] reply;")
			self.st.out("return &" + port + "Value;")
			self.vs.writeFunctionEnd();
			self.st.out("")
		if port_direction == rur.Direction.OUT:
			self.vs.writePortFunctionSignatureImpl(p, rur.Direction.OUT)
			self.st.out("// For now only int return values are supported")
#			self.st.out("if (!ReceiveGeneralRequest(" + portname + ", " + portname + "Ready, false)) return false;")
			self.st.out("std::stringstream ss; ss.clear(); ss.str(\"\");")
			if param_kind == idltype.tk_sequence:
				self.st.out("if (" + param_name + ".empty()) return true;")
				self.st.out(param_type + "::const_iterator it=" + param_name + ".begin();")
				self.st.out("ss << *it++;")
				self.st.out("for (; it!= " + param_name + ".end(); ++it)")
				self.st.inc_indent()
				self.st.out("ss << \" \" << *it;")
				self.st.dec_indent()
#				self.vs.writeFunctionEnd()
#				self.st.out('ss << "\0";')
			else:
				self.st.out("ss << " + param_name + "; // very dirty, no endianness, etc, just use the stream operator itself") 
#				self.st.out("ss << (" + param_name + "& 0xFF) << (" + param_name + ">> 8); // little-endian") 
			self.st.out("bool state = " + port + "Out.ready;")
#			self.st.out('std::cout << "sendrequest: " << ss.str() << " size: " << ss.str().size() << std::endl;')
			self.st.out("SendRequest(" + port + "Out.sock, state, false, ss.str());")
			self.st.out("if (state) " + port + "Out.ready = false;")
			self.st.out("if (!" + port + "Out.ready) {")
			self.st.inc_indent()
			self.st.out("bool ack_state = true;")
			self.st.out("ReceiveAck(" + port + "Out.sock, ack_state, true);")
			self.st.out("if (ack_state) {")
			self.st.inc_indent()
			self.st.out(port + "Out.ready = true;")
			self.st.out("return true;")
			self.vs.writeFunctionEnd()
			self.vs.writeFunctionEnd()
			self.st.out("return false;")
			self.vs.writeFunctionEnd()
			self.st.out("")


	def writeTick(self):
		self.st.out("HandleCommand();")

	def writeHelperFunctions(self):
		function_body = '''  /**
   * The resolve function can be called by modules to get a new socket (and if you want host name and port). It can also
   * be used by the connector, to bind to these previously set up sockets.
   */
  void Resolve(pns_record & record);
  void SendAck(zmq::socket_t *s, bool state);
  bool ReceiveAck(zmq::socket_t *s, bool & state, bool blocking);
  char* GetReply(zmq::socket_t *s, bool & state, bool blocking, int & reply_size);
  void SendRequest(zmq::socket_t *s, bool & state, bool blocking, std::string str);
  void HandleCommand();
  void Connect(std::string source, std::string target);
  zmq::socket_t* GetSocket(std::string name);
'''
		print function_body

	def writeHelperFunctionsImpl(self):
		function_body = '''
void ''' + self.vs.classname + '''::Resolve(pns_record & record) {
  std::cout << "Acquire TCP/IP port for " << record.name << std::endl;
  std::string reqname = record.name + ':' + record.pid;
  zmq::message_t request (reqname.size() + 1);
  memcpy ((void *) request.data (), reqname.c_str(), reqname.size());
  ns_socket->send(request);
  
  zmq::message_t reply;
  if (!ns_socket->recv (&reply)) return;
  size_t msg_size = reply.size();
  char* address = new char[msg_size+1];
  memcpy (address, (void *) reply.data(), msg_size);
  address[msg_size] = \'\\0\';
  std::string json = std::string(address);
  std::cout << "Received " << json << std::endl;
  delete [] address;
  
  // get json object
  bool valid;
  json_spirit::Value value;
  if (!json_spirit::read(json, value)) {
    valid = false;
    std::cerr << "Not a json value" << std::endl;
    return;
  }
  if (value.type() != json_spirit::obj_type) {
    std::cerr << "[1] Unexpected object type \"" << Value_type_str[value.type()] << "\"";
    std::cerr << " instead of \"" << Value_type_str[json_spirit::obj_type] << "\"" << std::endl;
    return;
  }
  
  // a "json_spirit object" is - by default - a vector of json pairs
  json_spirit::Object obj = value.get_obj();
  
  for( json_spirit::Object::size_type i = 0; i != obj.size(); ++i ) {
    const json_spirit::Pair& pair = obj[i];
    const std::string& key = pair.name_;
    const json_spirit::Value& value = pair.value_;
    if (key == "identifier") {
      // same thing
    } else if (key == "server") {
      record.host = value.get_str();
    } else if (key == "port") {
      record.port = value.get_str();
    } else if (key == "pid") {
      record.pid = value.get_str();
    }
  }
}

void ''' + self.vs.classname + '''::SendAck(zmq::socket_t *s, bool state) {
  if (debug) std::cout << "Send ACK" << std::endl;
  SendRequest(s, state, true, "ACK");
}

bool ''' + self.vs.classname + '''::ReceiveAck(zmq::socket_t *s, bool & state, bool blocking) {
  int reply_size = 0;
  char *reply = GetReply(s, state, blocking, reply_size);
  if (reply == NULL) return false;
  std::string req = std::string(reply);
  delete [] reply;
  if (req.find("ACK") != std::string::npos) {
    if (debug) std::cout << "Got ACK, thanks!" << std::endl;
    return true;
  }
  std::cerr << "Error: got \\"" << req << "\\", no ACK, state compromised" << std::endl;
  return false;
}
  
char* ''' + self.vs.classname + '''::GetReply(zmq::socket_t *s, bool & state, bool blocking, int & reply_size) {
  zmq::message_t reply;
  char* result = NULL;
  reply_size = 0;
  try {
    if (blocking)
      state = s->recv(&reply);
    else
      state = s->recv(&reply, ZMQ_DONTWAIT);
  } catch (zmq::error_t &e) {
    std::cout << "Error: received zmq::error_t " << e.what() << std::endl;
  }
  if (state) {
    size_t msg_size = reply.size();
    result = new char[msg_size+1];
    memcpy (result, (void *) reply.data(), msg_size);
    result[msg_size] = \'\\0\';
    reply_size = msg_size;
    //std::cout << "Result: \\"" << std::string(result) << "\\"" << std::endl;
  }
  return result;
}

void ''' + self.vs.classname + '''::SendRequest(zmq::socket_t *s, bool & state, bool blocking, std::string str) {
  if (state) {
    zmq::message_t request(str.size()+1);
    memcpy((void *) request.data(), str.c_str(), str.size()+1);
    if (debug) std::cout << "Send request: " << str << std::endl;
    if (blocking)
      state = s->send(request);
    else
      state = s->send(request, ZMQ_DONTWAIT);
  } else {
    std::cout << "Send nothing (still waiting to receive) " << std::endl;
  }
}

void ''' + self.vs.classname + '''::HandleCommand() {
  int reply_size = -1;
  bool state = false;
  char *reply = GetReply(cmd_socket, state, false, reply_size);
  if (reply == NULL) return;
  if (reply_size < 2) std::cerr << "Error: Reply is not large for magic header + command string" << std::endl;
  char magic_value = reply[0];
  reply[reply_size-1] = \'\\0\';
  if (magic_value == 0x01) { // connect to command...
    std::string name = std::string(reply+1);
    int pos = name.find("->");
    if (pos == std::string::npos) {
      std::cerr << "Error: no -> separator in connect command" << std::endl;
    }
    std::string source = name.substr(0, pos);
    std::string target = name.substr(pos+2); // todo: 
    std::cout << "Connect from " << source << " to " << target << std::endl;
    Connect(source, target);
  } else {
    std::cerr << "Error: Unknown command!" << std::endl;
  }
  SendAck(cmd_socket, true);
  delete [] reply;
}

void ''' + self.vs.classname + '''::Connect(std::string source, std::string target) {
  zmq::socket_t *s = GetSocket(source);
  pns_record t_record;
  t_record.name = "/resolve" + target;
  Resolve(t_record);
  std::stringstream ss; ss.clear(); ss.str("");
  ss << "tcp://" << t_record.host << ":" << t_record.port; 
  std::string sock = ss.str(); 
  std::cout << "Connect to socket " << sock << std::endl; 
  try {
    s->connect(sock.c_str());
  } catch (zmq::error_t &e) {
    std::cerr << "Error: Could not connect to " << target << ", because: " << e.what() << std::endl;
  }
}

zmq::socket_t* ''' + self.vs.classname + '''::GetSocket(std::string name) {
  for (int i = 0; i < zmq_sockets.size(); ++i) {
    if (zmq_sockets[i]->name.find(name) != std::string::npos) return zmq_sockets[i]->sock;
  }
  std::cerr << "Error: socket name could not be found!" << std::endl;
  return NULL;
  //assert(false); // todo, get the previously registered socket by name
}
'''
		print function_body
