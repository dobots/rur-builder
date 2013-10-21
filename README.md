# RurBuilder
RurBuilder is a generator of (AI) modules or agents for different types of middlewares. It is developed by the cloud robotics company DO, shorthand for Distributed Organisms B.V.. The letter combination RUR stands for Roughly Universal Robot platform but is meant to be a group of modules in the cloud and over smartphones for any purpose. In robotic communities there are many robotic platforms available, such as ROS, YARP, OROCOS, MARIE, ACT-R, SOAR, IDA. The first few concentrating on providing middleware functionalities like connectors, ports, messages and a hardware abstraction layer, the latter focusing on (cognitive) control architectures, a dynamic set of functional units providing the robot with features like sensor fusion, attention, anticipation, and context-awareness.

The RUR platform aims to provide a means to independently write your software modules from any middleware. It does not prescribe how a cognitive architecture should be organized. Neither does it prescribe the communication infrastructure. It only requires you to write short interface description files for your modules. These are taken by the rur-builder to generate the glue code for the currently supported middleware solutions. These interface description files can also be used by crawlers on the web and be able to bootstrap distribution architectures for software modules (above the distribution frameworks for apps).

## Middleware support
RurBuilder uses omniIDL and supports the following middlewares:

1. YARP, Yet Another Robot Platform
2. ROS, Robotic Operating System
3. ZeroMQ, ZeroMQ is the middleware used in the Storm data processing topology of Twitter
4. AgentScape (to be implemented)
5. Android (using the android messenger class)
6. NodeJS (wrapper)

## Abstraction support
The abstractions are written in IDL and look like a Java or C++ interface. The current abstractions allow for:

- incoming and outgoing ports, without the need to specify that they will be buffered ports in YARP, or publish/subscribe channels in ROS, etc. 
- automatic translation of custom structs to middleware-specific structs (preliminary)
- definition of input parameters on the command-line
- namespaces

## Installation
RurBuilder uses omniIDL, so firstly omniIDL has to be installed:
```bash
sudo apt-get install omniidl
```

The rur-builder itself can be installed by:
```bash
cd rur-builder
sudo make install
```

## Usage
The rur-builder is a commandline tool with the following options:
```bash
    -h        Show this help message
    -i arg    IDL file (example: ~/MyModule/aim-config/MyModule.idl)
    -p arg    Path to the backends (example: /usr/share/rur/backends)
    -o arg    Output file (example: ~/MyModule/aim-core/inc/MyModule.h)
    -b arg    Backend (example: rur_main_head)
    -m arg    Middleware (default: standard)
    -v        Be more verbose
```
The different backends create different types of files.
- rur_main_head Creates a header file (.h) for the module base class.
- rur_main_impl Creates an implementation file (.cpp) for the module base class.
- rur_main_relay Creates an implementation file that copies data from ports in to ports out. This is handy to create modules that split, merge or convert data. This currently only works in combination with the [AIM tools](https://github.com/dobots/aimtools). Usage of this backend can be found below.

Typical usage:
```bash
rur-builder -i MyModule.idl -p /usr/share/rur/backends -b rur_main_head -m yarp -o MyModule.h
rur-builder -i MyModule.idl -p /usr/share/rur/backends -b rur_main_impl -m yarp -o MyModule.cpp
```

Optional port options:
By adding a comment "// middleware <middleware>" above the a port definition in the IDL file, the middleware for that port can be specified. Example:
```c++
// middleware yarp
// Input from microphone in the form of an array
void Audio(in long_seq input);
```

## Design Cycle with RUR
The common design cycle is like this:

- Write an .idl file for your module, which can be a new or existing module. The module can contain several files, say we have "esn.cpp".
- Run the rur-builder using the standard middleware to generate a C++ header and implementation file "rur_esn.h" and "rur_esn.cpp.
- We make the "esn.cpp" file network-aware by making use of the read/write port functions defined in the "rur_esn.h" file.
- Run the rur-builder in a middleware-specific mode. For example we run a yarp server on the robot. Generate "rur_esn.h" and "rur_esn.cpp" using "yarp" as middleware. The files will now be rewritten by yarp-specific function calls. Nothing needs to be changed in the "esn.cpp" file.
- Run the rur-builder for another type of middleware, e.g. ROS. Now instead of the function calls for yarp, the files will contain code for ros.

## Alternatives
Programming code is separated into different modules. For example a "particle filter", an "echo state network", a "reinforcement learning module", or other type of modules that provide algorithms required for robotic applications. Each module is described in a language-independent manner using a well-known variant of the CORBA IDL, interface description language. An IDL is normally used to have a language neural interface between components, such that these can be written in multiple languages. Specific instances of IDLs (besides the CORBA one) are Protocol Buffers (Google), Avro (Hadoop, Apache), Thrift (Facebook, Apache), and WSDL (tailored to web services).

The RUR platform provides the glue using the IDL specification of a component to generate a language-specific header file, which can be used to use this component in a certain middleware. So, from an IDL specification, code is generated that allows a component written in C to be used on a robot running the YARP middleware. However, using this same IDL specification a component written in Java can be used within the JADE multi-agent system. Of course, if the programmer allows a component to be used from within Java or C - by SWIG e.g. - this will make it possible to use the same component in either programming language, but this is not the task of the RUR platform.

The RUR platform is similar to a recent effort, Genom3, generator of modules. However, contrary to Genom3 which decorates existing code with generic keywords that will be replaced by middleware-specific terms, the component code will not be touched by the RUR compiler. This means that the code is still compilable, syntax highlighting still works properly, and declarations and references can be found by the indexer (in e.g. Eclipse).

## Example
An example that shows the first automatically generated structures, instances, and functions of a so-called ARTMAP module to be used in YARP middleware:

![alt text](https://github.com/dobots/rur-builder/raw/master/doc/rur_idl2yarp.jpg "IDL to YARP example")

## Relay backend
When using the rur_main_relay backend, the whole module is supposed to be generated. This is handy to generate modules that split, merge or convert ports. To specify which input ports should be relayed to which output ports, the port name is used in the form: "<name>__<In/Out>[_<N>]". Ports with the same name are coupled: data read from "name__In" will be written to "name__Out". Merging and splitting is done by appending number N.
Examples:
```C++
// Convert port from long to float
void Audio__In(in long input);
void Audio__Out(out float output);

// Merge ports and convert long to float
void Sample__In_0(in long input);
void Sample__In_1(in long input);
void Sample__Out(out float output);

// Split port
void Temperature__In(in long input);
void Temperature__Out_0(out long output);
void Temperature__Out_1(out long output);
void Temperature__Out_2(out long output);
```

## Copyrights
The copyrights (2012) belong to:

- Author: Anne van Rossum
- Almende B.V., http://www.almende.com and DO bots B.V., http://www.dobots.nl
- Rotterdam, The Netherlands
