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

Simply copy the canfix.c and canfix.h files into your project and compile them.
  

-----------------
Use
-----------------

libcanfix is CAN is completely agnostic of the CAN implementation.  Callback
functions are used by the library to send CAN data and it it up to the user
to program these functions for the CAN device that is being used

Incomgin CAN messages are passed into the library as arguments to the
canfix_exec function.  The canfix_exec function determines what kind of CANFiX
message has been received and either handles it internally or calls a callback
so that the main program can handle it.


