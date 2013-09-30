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
import sys, string

sys.path.append('./helper')
from helper import rur
from rur_main_visitor import MainVisitor
from rur_main import Main


class Main_head (Main):
	def writeAll(self):
		self.vs.writeFileComment()
		
		self.vs.writeIncludeGuardStart()
		
		# INCLUDES
		self.vs.writeIncludes()
		for k, b in self.beList.iteritems():
			b.writeIncludes()
		self.st.out("")
		
		self.vs.writeNamespaceStart()
		
		# BEFORE CLASS
		self.vs.writeBeforeClassVarsDecl()
		self.vs.writeBeforeClassFuncsDecl()
		for k, b in self.beList.iteritems():
			b.writeBeforeClassVarsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeBeforeClassPortVarsDecl(p)
		for k, b in self.beList.iteritems():
			b.writeBeforeClassFuncsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeBeforeClassPortFuncsDecl(p)
		
		self.beList[self.beStrDef].writeClassStart() # in case it needs to be derived from some backend base class
		
		# PRIVATE
		self.st.out("private:")
		self.st.inc_indent()

		self.vs.writePrivateVarsDecl()
		self.vs.writePrivateFuncsDecl()
		for k, b in self.beList.iteritems():
			b.writePrivateVarsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writePrivatePortVarsDecl(p)
		for k, b in self.beList.iteritems():
			b.writePrivateFuncsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writePrivatePortFuncsDecl(p)
		
		
		# PROTECTED
		self.st.dec_indent()
		self.st.out("protected:")
		self.st.inc_indent()
		
		self.vs.writeProtectedVarsDecl()
		self.vs.writeProtectedFuncsDecl()
		for k, b in self.beList.iteritems():
			b.writeProtectedVarsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeProtectedPortVarsDecl(p)
		for k, b in self.beList.iteritems():
			b.writeProtectedFuncsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writeProtectedPortFuncsDecl(p)
		
		
		# PUBLIC
		self.st.dec_indent()
		self.st.out("public:")
		self.st.inc_indent()
		self.vs.writePublicVarsDecl()
		self.vs.writePublicFuncsDecl()
		for k, b in self.beList.iteritems():
			b.writePublicVarsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writePublicPortVarsDecl(p)
		for k, b in self.beList.iteritems():
			b.writePublicFuncsDecl()
		for p in self.vs.portList:
			self.beList[p.beStr].writePublicPortFuncsDecl(p)
		
		
		self.vs.writeClassEnd()
		self.vs.writeNamespaceEnd()
		self.vs.writeIncludeGuardEnd()


# Initialize this parser
def run(tree, args):
	st = output.Stream(sys.stdout, 2)
	main = Main_head(st, tree, args)
	
	# And then write everything to the to-be-generated header file
	main.writeAll()
