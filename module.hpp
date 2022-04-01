#include <vector>
#include <string>
#include <memory>

static const std::vector<const char*> prompts = {
    // "Bottom of ramp",
    "Top of ramp",
    "Behind lever 0",
    "Behind lever 1",
    //"Behind lever 2",
    "Behind burger flip",
    "On jukebox light"      // robot faces 180 degrees, CDS is on top of light
};

static const size_t nprompts = prompts.size();

class ModuleProvider;

class Module {
protected:
    friend ModuleProvider;
    Module() {}
public:
    virtual ~Module() {}
    virtual const std::string &name() const = 0;
    virtual int run() = 0;
};

class ModuleProvider {
private:
    ModuleProvider();
    std::vector<std::unique_ptr<Module>> _vec;
    void register_module(std::unique_ptr<Module>&&);
public:
    static ModuleProvider &instance();
    const std::vector<std::unique_ptr<Module>> &vec();
};

static ModuleProvider &module_provider = ModuleProvider::instance();

class RunCourseModule : public Module {
public:
    const std::string &name() const;
    int run();
};

class CalibrationModule : public Module {
public:
    const std::string &name() const;
    int run();
};

class CDSModule : public Module {
public:
    const std::string &name() const;
    int run();
};