#ifndef __MODULE_LOADER_H_INCLUDED__
#define __MODULE_LOADER_H_INCLUDED__

#include <ros/ros.h>
#include "modules/module.h"

using namespace std;

class ModuleLoader {
	private:
		static std::map<std::string, Module *> modules;

	public:
		void init();
		static void add(string name, Module * module);
		static Module * get(string name);

};

#endif