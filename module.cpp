#include <module.hpp>

ModuleProvider &ModuleProvider::instance() {
    static ModuleProvider inst;
    return inst;
}

void ModuleProvider::register_module(std::unique_ptr<Module> &&fac) {
    _vec.emplace_back(std::move(fac));
}

ModuleProvider::ModuleProvider() {
    register_module(std::make_unique<RunCourseModule>());
    register_module(std::make_unique<CalibrationModule>());
    register_module(std::make_unique<CDSModule>());
}

const std::vector<std::unique_ptr<Module>> &ModuleProvider::vec() {
    return _vec;
}