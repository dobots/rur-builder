# -*- python -*-
#                           Package   : omniidl
# main_head.py              Created on: 2013/10/29
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
#   Android back-end for network abstractions 

"""Android service IDL compiler back-end."""

from omniidl import idlast, idltype, idlutil, idlvisitor, output
import sys, string

sys.path.append('./helper')
from helper import rur
from rur_main_visitor import MainVisitor
from rur_main import Main


class Android_service (Main):
	def writeAll(self):
		self.writePackage()
		self.writeImports()
		
		self.st.out("public class " + self.vs.classname + "Service extends Service {")
		self.st.inc_indent()
		self.st.out("private static final String TAG = \"" + self.vs.classname + "Service\";")
		self.st.out("private static final String MODULE_NAME = \"" + self.vs.classname + "\";")
		self.st.out("private int mId = -1;")
		self.st.out("Messenger mToMsgService = null;")
		self.st.out("final Messenger mFromMsgService = new Messenger(new IncomingMsgHandler());")
		self.st.out("boolean mMsgServiceIsBound;")
		self.st.out("")
		
		for p in self.vs.portList:
			if (p.beStr == "android"):
				self.writePortDeclaration(p)
		self.st.out("")
		
		for p in self.vs.portList:
			if (p.beStr == "android"):
				self.writePortMessageHandler(p)
		
		self.writeServiceConnection()
		self.writeIncomingMsgHandler()
		self.writeBody()
		self.writeAimRun()
		self.st.out("// static constructor")
		self.st.out("static {")
		self.st.inc_indent()
		self.st.out("System.loadLibrary(\"" + self.vs.classname + "\");")
		self.vs.writeFunctionEnd()
		self.vs.writeFunctionEnd()
		

	def writePackage(self):
		self.st.out("package org.dobots." + self.vs.classname.lower() + ".aim;")

	def writeImports(self):
		body = '''
import java.util.ArrayList;
import java.util.List;

import android.app.Service;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.Messenger;
import android.os.RemoteException;
import android.util.Log;
'''
		print body

	def writeBody(self):
		body = '''
	AIMRun mAIMRun = null;
	
	public void start() {
		if (mId < 0)
			return;
		if (!mMsgServiceIsBound)
			bindToMsgService();
		if (mAIMRun == null) {
			mAIMRun = new AIMRun();
			mAIMRun.execute(mId);
		}
	}
	
	public void onCreate() {
		// On create mId is unknown!
	}
	
	public void onDestroy() {
		super.onDestroy();
		mAIMRun.cancel(true);
		unbindFromMsgService();
		Log.d(TAG, "onDestroy");
	}
	
	@Override
	public IBinder onBind(final Intent intent) {
		Log.i(TAG,"onBind: " + intent.toString());
		return null; // No binding provided
//		return mFromMsgService.getBinder();
	}
	
	// Called when all clients have disconnected from a particular interface of this service.
	@Override
	public boolean onUnbind(final Intent intent) {
		return super.onUnbind(intent);
	}
	
	// Deprecated since API level 5 (android 2.0)
	@Override
	public void onStart(Intent intent, int startId) {
//		handleStartCommand(intent);
		mId = intent.getIntExtra("id", 0);
		Log.d(TAG, "onStart " + mId);
		start();
	}
	
	// Called each time a client uses startService()
	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
//		handleStartCommand(intent);
		// We want this service to continue running until it is explicitly stopped, so return sticky.
		mId = intent.getIntExtra("id", 0);
		Log.d(TAG, "onStartCommand " + mId);
		start();
		return START_STICKY;
	}
	
	void bindToMsgService() {
		// Establish a connection with the service.  We use an explicit class name because there is no reason to be 
		// able to let other applications replace our component.
		Intent intent = new Intent();
		intent.setClassName("org.dobots.dodedodo", "org.dobots.dodedodo.MsgService");
		bindService(intent, mMsgServiceConnection, Context.BIND_AUTO_CREATE);
		mMsgServiceIsBound = true;
		Log.i(TAG, "Binding to msgService");
	}
	
	void unbindFromMsgService() {
		if (mMsgServiceIsBound) {
			// If we have received the service, and registered with it, then now is the time to unregister.
			if (mToMsgService != null) {
				Message msg = Message.obtain(null, AimProtocol.MSG_UNREGISTER);
				Bundle bundle = new Bundle();
				bundle.putString("module", MODULE_NAME);
				bundle.putInt("id", mId);
				msg.setData(bundle);
				msgSend(msg);
			}
			// Detach our existing connection.
			unbindService(mMsgServiceConnection);
			mMsgServiceIsBound = false;
			Log.i(TAG, "Unbinding from msgService");
		}
	}
	
	// Send a msg to the msgService
	protected void msgSend(Message msg) {
		if (!mMsgServiceIsBound) {
			Log.i(TAG, "Can't send message to service: not bound");
			return;
		}
		try {
			msg.replyTo = mFromMsgService;
			mToMsgService.send(msg);
		} catch (RemoteException e) {
			Log.i(TAG, "Failed to send msg to service. " + e);
			// There is nothing special we need to do if the service has crashed.
		}
	}
	
	// Send a msg to some messenger
	protected void msgSend(Messenger messenger, Message msg) {
		if (messenger == null || msg == null)
			return;
		try {
			//msg.replyTo = mFromMsgService;
			messenger.send(msg);
		} catch (RemoteException e) {
			Log.i(TAG, "failed to send msg. " + e);
			// There is nothing special we need to do if the service has crashed.
		}
	}
	
'''
		print body

	def writeServiceConnection(self):
		self.st.out("private ServiceConnection mMsgServiceConnection = new ServiceConnection() {")
		self.st.inc_indent()
		self.st.out("public void onServiceConnected(ComponentName className, IBinder service) {")
		self.st.inc_indent()
		self.st.out("// This is called when the connection with the service has been established, giving us the service object")
		self.st.out("// we can use to interact with the service.  We are communicating with our service through an IDL")
		self.st.out("// interface, so get a client-side representation of that from the raw service object.")
		self.st.out("mToMsgService = new Messenger(service);")
		self.st.out("Message msg = Message.obtain(null, AimProtocol.MSG_REGISTER);")
		self.st.out("Bundle bundle = new Bundle();")
		self.st.out("bundle.putString(\"package\", getPackageName());")
		self.st.out("bundle.putString(\"module\", MODULE_NAME);")
		self.st.out("bundle.putInt(\"id\", mId);")
		self.st.out("msg.setData(bundle);")
		self.st.out("msgSend(msg);")
		
		for p in self.vs.portList:
			if (p.beStr == "android"):
				port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
				if port_direction == rur.Direction.IN:
					self.st.out("{")
					self.st.inc_indent()
					self.st.out("Message msgPort = Message.obtain(null, AimProtocol.MSG_SET_MESSENGER);")
					self.st.out("msgPort.replyTo = mPort" + port_name + "InMessenger;")
					self.st.out("Bundle bundlePort = new Bundle();")
					self.st.out("bundlePort.putString(\"module\", MODULE_NAME);")
					self.st.out("bundlePort.putInt(\"id\", mId);")
					self.st.out("bundlePort.putString(\"port\", \"" + port_name.lower() + "\");")
					self.st.out("msgPort.setData(bundlePort);")
					self.st.out("msgSend(mToMsgService, msgPort);")
					self.vs.writeFunctionEnd()
					self.st.out("")
		
		self.st.out("")
		self.st.out("Log.i(TAG, \"Connected to MsgService: \" + mToMsgService.toString());")
		self.vs.writeFunctionEnd()
		self.st.out("public void onServiceDisconnected(ComponentName className) {")
		self.st.inc_indent()
		self.st.out("// This is called when the connection with the service has been unexpectedly disconnected: its process crashed.")
		self.st.out("mToMsgService = null;")
		self.st.out("Log.i(TAG, \"Disconnected from MsgService\");")
		self.vs.writeFunctionEnd()
		self.st.dec_indent()
		self.st.out("};")
		self.st.out("")

	def writeIncomingMsgHandler(self):
		self.st.out("// Handle messages from MsgService")
		self.st.out("class IncomingMsgHandler extends Handler {")
		self.st.inc_indent()
		print "    @Override"
		self.st.out("public void handleMessage(Message msg) {")
		self.st.inc_indent()
		self.st.out("switch (msg.what) {")
		self.st.out("case AimProtocol.MSG_SET_MESSENGER:")
		self.st.inc_indent()
		
		# set ports here
		for p in self.vs.portList:
			if (p.beStr == "android"):
				port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
				if port_direction == rur.Direction.OUT:
					self.st.out("if (msg.getData().getString(\"port\").equals(\"" + port_name.lower() + "\"))")
					self.st.inc_indent()
					self.st.out("mPort" + port_name + "OutMessenger = msg.replyTo;")
					self.st.dec_indent()
		
		self.st.out("break;")
		self.st.dec_indent()
		self.st.out("case AimProtocol.MSG_STOP:")
		self.st.inc_indent()
		self.st.out("Log.i(TAG, \"stopping\");")
		self.st.out("stopSelf();")
		self.st.out("break;")
		self.st.dec_indent()
		self.st.out("default:")
		self.st.inc_indent()
		self.st.out("super.handleMessage(msg);")
		self.vs.writeFunctionEnd()
		self.vs.writeFunctionEnd()
		self.vs.writeFunctionEnd()
		self.st.out("")

	def writePortDeclaration(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("Messenger mPort" + port_name + "InMessenger = new Messenger(new Port" + port_name + "MessengerHandler());")
			self.st.out("private List<" + self.getJavaType(param_type, param_kind) + "> mPort" + port_name + \
				"InBuffer = new ArrayList<" + self.getJavaType(param_type, param_kind) + ">(0);")
		if port_direction == rur.Direction.OUT:
			self.st.out("Messenger mPort" + port_name + "OutMessenger = null;")

	def writePortMessageHandler(self, p):
		port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
		if port_direction == rur.Direction.IN:
			self.st.out("class Port" + port_name + "MessengerHandler extends Handler {")
			self.st.inc_indent()
			print "    @Override"
			self.st.out("public void handleMessage(Message msg) {")
			self.st.inc_indent()
			self.st.out("switch (msg.what) {")
			self.st.out("case AimProtocol.MSG_PORT_DATA:")
			self.st.inc_indent()
			
			if param_kind == idltype.tk_sequence:
				self.st.out(self.getMessengerType(param_type, param_kind) + " readVal = msg.getData()." + self.getMessengerGetType(param_type, param_kind) + "(\"data\");")
				self.st.out(self.getJavaType(param_type, param_kind) + " bufVal = new " + self.getJavaType(param_type, param_kind) + "(readVal.length);")
				
				# Just for debug! should remove this for speedup
				self.st.out("")
				self.st.out("// Debug")
				self.st.out("StringBuffer str = new StringBuffer(\"Read msg: \");")
				self.st.out("for (int i=0; i<readVal.length; i++) {")
				self.st.inc_indent()
				self.st.out("bufVal.set(i, readVal[i]);")
				self.st.out("str.append(readVal[i]);")
				self.st.out("str.append(\" \");")
				self.vs.writeFunctionEnd()
				self.st.out("Log.d(TAG, str.toString());")
				self.st.out("")
				
				self.st.out("synchronized(mPort" + port_name + "InBuffer) {")
				self.st.inc_indent()
				self.st.out("mPort" + port_name + "InBuffer.add(bufVal);")
			else:
				self.st.out(self.getJavaType(param_type, param_kind) + " readVal = msg.getData()." + self.getMessengerGetType(param_type, param_kind) + "(\"data\");")
				
				# Just for debug! should remove this for speedup
				self.st.out("// Debug")
				self.st.out("Log.d(TAG, \"Read msg: \" + readVal);")
				
				self.st.out("synchronized(mPort" + port_name + "InBuffer) {")
				self.st.inc_indent()
				self.st.out("mPort" + port_name + "InBuffer.add(readVal);")
			self.vs.writeFunctionEnd()
			self.st.dec_indent()
			self.st.out("default:")
			self.st.inc_indent()
			self.st.out("super.handleMessage(msg);")
			self.vs.writeFunctionEnd()
			self.vs.writeFunctionEnd()
			self.vs.writeFunctionEnd()
			self.st.out("")

	def writeAimRun(self):
		self.st.out("// AsyncTask<Params, Progress, Result>")
		self.st.out("private class AIMRun extends AsyncTask<Integer, Void, Boolean> {")
		self.st.inc_indent()
		self.st.out("protected Boolean doInBackground(Integer... id) {")
		self.st.inc_indent()
		self.st.out("Log.i(TAG, \"Starting AIMRun \" + id);")
		self.st.out(self.vs.classname + "Ext aim = new " + self.vs.classname + "Ext();")
		self.st.out("//aim.Init(\"0\"); // TODO: pass on the id")
		
		for p in self.vs.portList:
			if (p.beStr == "android"):
				port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
				if port_direction == rur.Direction.OUT:
					self.st.out("Android" + port_name + "Read_t output" + port_name + ";")
		
		self.st.out("")
		self.st.out("while (true) {")
		self.st.inc_indent()
		for p in self.vs.portList:
			if (p.beStr == "android"):
				port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
				if port_direction == rur.Direction.IN:
					self.st.out("synchronized(mPort" + port_name + "InBuffer) {")
					self.st.inc_indent()
					self.st.out("if (!mPort" + port_name + "InBuffer.isEmpty()) {")
					self.st.inc_indent()
					self.st.out("aim.androidWrite" + port_name + "(mPort" + port_name + "InBuffer.get(0));")
					self.st.out("mPort" + port_name + "InBuffer.remove(0);")
					self.vs.writeFunctionEnd()
					self.vs.writeFunctionEnd()
					self.st.out("")
		
		self.st.out("aim.Tick();")
		
		for p in self.vs.portList:
			if (p.beStr == "android"):
				port, port_name, port_direction, param_name, param_type, param_kind, port_pragmas, port_comments = self.vs.getPortConfiguration(p)
				if port_direction == rur.Direction.OUT:
					self.st.out("output" + port_name + " = aim.androidRead" + port_name + "();")
					self.st.out("if (output" + port_name + ".getSuccess()) {")
					self.st.inc_indent()
					if param_kind == idltype.tk_sequence:
						
						# Just for debug! should remove this for speedup
						self.st.out("")
						self.st.out("// Debug")
						self.st.out("StringBuffer str = new StringBuffer();")
						self.st.out("str.append(\"output" + port_name + "=\");")
						self.st.out("str.append(output" + port_name + ".getVal().toString());")
						self.st.out("str.append(\" \");")
						#self.st.out("Log.d(TAG, \"output" + port_name + "=\" + output" + port_name + ".getVal().toString() + \" \");")
						size = "(int) output" + port_name + ".getVal().size()"
						self.st.out(self.getMessengerType(param_type, param_kind) + " data = new " + self.getMessengerType(param_type, param_kind, size) + ";")
						self.st.out("for (int i=0; i<output" + port_name + ".getVal().size(); i++) {")
						self.st.inc_indent()
						self.st.out("data[i] = output" + port_name + ".getVal().get(i);")
						self.st.out("str.append(output" + port_name + ".getVal().get(i));")
						self.st.out("str.append(\" \");")
						#self.st.out("Log.d(TAG, output" + port_name + ".getVal().get(i) + \" \");")
						self.vs.writeFunctionEnd()
						self.st.out("Log.d(TAG, str.toString());")
						self.st.out("")
						
						self.st.out("Message msg = Message.obtain(null, AimProtocol.MSG_PORT_DATA);")
						self.st.out("Bundle bundle = new Bundle();")
						self.st.out("bundle.putInt(\"datatype\", " + self.getMessengerDataType(param_type, param_kind) + ");")
						self.st.out("bundle." + self.getMessengerPutType(param_type, param_kind) + "(\"data\", data);")
						self.st.out("msg.setData(bundle);")
						self.st.out("msgSend(mPort" + port_name + "OutMessenger, msg);")
					else:
						
						# Just for debug! should remove this for speedup
						self.st.out("// Debug")
						self.st.out("Log.d(TAG, \"output" + port_name + "=\" + output" + port_name + ".getVal());")
						
						self.st.out("Message msg = Message.obtain(null, AimProtocol.MSG_PORT_DATA);")
						self.st.out("Bundle bundle = new Bundle();")
						self.st.out("bundle.putInt(\"datatype\", " + self.getMessengerDataType(param_type, param_kind) + ");")
						self.st.out("bundle." + self.getMessengerPutType(param_type, param_kind) + "(\"data\", output" + port_name + ".getVal());")
						self.st.out("msg.setData(bundle);")
						self.st.out("msgSend(mPort" + port_name + "OutMessenger, msg);")
					self.vs.writeFunctionEnd()
					self.st.out("")
		
		self.st.out("if (isCancelled()) break;")
		self.vs.writeFunctionEnd()
		self.st.out("return true;")
		self.vs.writeFunctionEnd()
		self.st.out("")
		self.st.out("protected void onCancelled() {")
		self.st.inc_indent()
		self.st.out("Log.i(TAG, \"Stopped AIMRun\");")
		self.vs.writeFunctionEnd()
		self.vs.writeFunctionEnd()
		self.st.out("")

	# Might need to update this for more types
	def getJavaType(self, param_type, param_kind):
		if param_kind == idltype.tk_sequence:
			type = self.vs.getSeqType(param_type)
			return "vector_" + type
		else:
			if param_type == "int":
				return "Integer"
			return param_type[0].upper() + param_type[1:]

	def getMessengerType(self, param_type, param_kind, size=""):
		if param_kind == idltype.tk_sequence:
			return self.vs.getSeqType(param_type) + "[" + size + "]"
		else:
			return param_type

# TODO: maybe use ArrayList instead?
	def getMessengerGetType(self, param_type, param_kind):
		if param_kind == idltype.tk_sequence:
			seqType = self.vs.getSeqType(param_type)
			return "get" + seqType[0].upper() + seqType[1:] + "Array"
		else:
			if (param_type == "string"):
				return "getString"
			return "get" + param_type[0].upper() + param_type[1:]

	def getMessengerPutType(self, param_type, param_kind):
		if param_kind == idltype.tk_sequence:
			seqType = self.vs.getSeqType(param_type)
			return "put" + seqType[0].upper() + seqType[1:] + "Array"
		else:
			if (param_type == "string"):
				return "putString"
			return "put" + param_type[0].upper() + param_type[1:]

	def getMessengerDataType(self, param_type, param_kind):
		if param_kind == idltype.tk_sequence:
			seqType = self.vs.getSeqType(param_type)
			if (seqType == "int"):
				return "AimProtocol.DATATYPE_INT_ARRAY"
			return "AimProtocol.DATATYPE_FLOAT_ARRAY"
		else:
			if (param_type == "string"):
				return "AimProtocol.DATATYPE_STRING"
			if (param_type == "int"):
				return "AimProtocol.DATATYPE_INT"
			return "AimProtocol.DATATYPE_FLOAT"

# Initialize this parser
def run(tree, args):
	st = output.Stream(sys.stdout, 2)
	main = Android_service(st, tree, args)
	
	# And then write everything to the to-be-generated header file
	main.writeAll()
