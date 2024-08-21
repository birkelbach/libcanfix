***************************************************************
C CAN-FIX Library
***************************************************************

----------------
Overview
----------------

CAN-FIX is the CAN bus implementation of a set of protocols and specifications
known as FIX. FIX stands for Flight Information eXchange and is an attempt to
standardize communication among aircraft systems in small aircraft.

This library is made to make handling CANFiX data easier.  It's mostly intended
to be used in embedded projects as a source file include.  It can be compiled
and used on desktop computers as well and this is useful for running the test
suite.

The library is currently very simple and is mainly used for message dispatching.
Features to aid in handling specific types of messages will be added in the future.

-----------------
Installation
-----------------

For embedded applications simply copy the canfix.c and canfix.h files into your
project and compile them.

Cmake can also be used to compile the library as well as run tests against it.

-----------------
Use
-----------------

libcanfix is completely agnostic of the CAN implementation.  Callback
functions are used by the library to send CAN data and it it up to the user
to program these functions for the CAN device that is being used.

I tend to create a git submodule in my source trees so that when improvements are
made they can be easily distributed to all of my projects.  This library is still
under development so breaking changes are likely.  Proceed with caution.

Multiple CANFiX networks can be handled simultaneously.  Each connection is represented
by a canfix_object.  A pointer to this object is passed to most functions in the
library.  This object needs to be allocated by the user code and a pointer passed to
the canfix_init() function along with the initial node number, device type and
model number.

A textual description of the node can be passed to the canfix_set_description()
function.

After the object is set up, the callback functions need to be set.  The most important
callback function is the write callback.  This function is called when the library
needs to send a CAN frame on the network.

Other callback functions can be set that will be called by the library when certain
messages are received on the network.  Some received messages are automatically
handled by the library.

Incoming CAN messages are passed into the library as arguments to the
canfix_exec function.  The canfix_exec function determines what kind of CANFiX
message has been received and either handles it internally or calls a callback
so that the main program can handle it.

It is typically not a good idea to call canfix_exec() from an interrupt routine
since it's likely that response messages may be sent during the execution of
canfix_exec().  There is a convenience FIFO queue built into the library
that can be used to store frames that are retrieved in an interrupt routine.
The frames can be retrieved later by the main loop and handled there.  The queue
can be removed with preprocessor directives in the canfix.h file if it is not
needed.  The size of the queue can also be set there.

See the source code for details on each of these functions.  I'll get around to
doing a proper job on this documentation at some point, but right now it's pretty
immature and I'm still working out the kinks.

------------------
License
------------------

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA


