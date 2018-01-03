#include "module_loader.h"

void ModuleLoader::add(string name, Module * module) {
	ModuleLoader::modules.insert(std::pair<std::string, Module *>(name, module));
}

// example:		Button * button = static_cast<Button *>(moduleLoader.get("button"));
Module * ModuleLoader::get(string name) {
	return ModuleLoader::modules[name];
}

void ModuleLoader::performReadings() {
	 // call read() function of every module
	for (std::map<std::string, Module *>::iterator moduleIterator = modules.begin(); moduleIterator != modules.end(); ++moduleIterator) {
		moduleIterator->second->read();
	}
}