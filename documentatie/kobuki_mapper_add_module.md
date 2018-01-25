# kobuki_mapper - Add module
This document describes how you can create & use your own module and add it to the node 'kobuki_mapper'.

## Create files
First step is to create the necessary files. Think of how you want to name your module. It should be short, most likely 
noun and must describe what the module does. Also make sure the module name is not in use already.

Now you can create the files for your module to put your logic in. Every module requires a header and source file. 
These you can put inside ```kobuki_mapper/src/modules/```. You should create the following files:
 
 ```
 demo.cpp
 demo.h
 ```
 
 Replace 'demo' with the name of your module. If your module name contain spaces your should replace these with 
 underscores (also called snake_case). This is the only place you should use these, on other places it would be CamelCase.
 
 ## Write your module
 
 ### Header file
 
 Start by creating the header file. This contains the same as any other C++ program, only having a basic structure.
 First you have to include the header file of the 'module'-component:
  
 ```
#include "module.h"
 ```
 
 It is also recommended to use an 'include guard' (using the filename as define value, but UPPER-CASE):
 
```
#ifndef __DEMO_H_INCLUDED__
#define __DEMO_H_INCLUDED__
  
#include "module.h"
 
#endif
```

Now after the include of the header you have to define your class. This class inheritance the Module-base class. Replace 
'Demo' with your own module-name.
```
#include "module.h"

class Demo : public Module {

};
 ```

We are almost done with the header file, you just have to add some methods which are defined in the base-module but must 
be overwritten. It is important to add these otherwise you can get (strange) errors.

```
#include "module.h"

class Demo : public Module {
	public:
		Demo(ros::NodeHandle * nodeHandle);
		void read();
};
 ```
 
As you can read we create a constructor getting a 'nodeHandle' as argument. You can use this 'nodeHandle' to subscribe 
or advertise on the 'main'-handle. The 'read'-method is a function which get called every loop in the spin function. It 
is possible to create some logic in here based on time.

You can add anything you want from here.

### Source file

The source-file is straight forward if you look on how the header-file is created. There is no requirement on how you
 fill in the methods. It is recommended to:
  
* Create subscribers on topics in your constructor. You should create a 'private' variable in the header file to store these.
* For debug purposes, write all logging this way: ```ROS_INFO_STREAM("Demo:: Module initialized.");``` (replace 'Demo' 
with your module name and it is recommended to add this line of code in the constructor.)


## Build your module

Now you have written your module you have to make sure it gets build. You can do this by adding your module to the 
'CMakeLists.txt'. Just add the source file after the following lines:
```
add_executable(kobuki_mapper
	src/kobuki_mapper.cpp
	src/module_loader.cpp
	src/modules/module.cpp
	src/modules/demo.cpp
	...
```

Now you are ready to use it.

## Use your module

You can use your module in the 'kobuki_mapper'-node after initalizing the 'nodeHandle'. (see ```main()```-method in 
kobuki_mapper.cpp). You should add your module using the 'ModuleLoader'. Like this:
```
ModuleLoader::add("demo", new Demo(&n));
```

The first argument is the name of your module, just name it the way you created the header/source files. The second argument
calls your class and gets the 'nodeHandle' as argument.

Also don't forget to include your header file, otherwise the class cannot be found:
```
#include "modules/demo.h"
```
(on top of kobuki_mapper.cpp, replace 'demo' with your own module name.)

Now you are done with initializing your module. You can now use it two ways:

Short-hand:
```
(static_cast<Demo *>(ModuleLoader::get("demo")))->method()
```

This line of code can be put in if-statements or just to make a simple call. It returns what the 'method()'-method returns.
So you can for example create a method which returns a boolean and use the above code inside an if-statement.

If you have to talk multiple times to your module you can call it this way:

```
Demo * demo = (static_cast<Demo *>(ModuleLoader::get("demo")));
demo->method();
demo->isActivated(true);
demo->close();
```

So you define a variable which will hold the pointer to your module.


# Last notes

Important is naming your files, classes, methods the right way.

* snake_case files (test_file.cpp)
* CamelCase class-names (ClassName)
* camelCase methods. (getUser)

Other things:

* Create subscriber in class this way: 
```nodeHandle->subscribe("/mobile_base/events/button", 100, &Button::buttonPressCallback, this);```

