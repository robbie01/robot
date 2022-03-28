#include <vector>
#include <string>
#include <memory>

static const char *prompts[] = {
//    "Burger flip",
    "Behind burger flip",
    "Bottom of ramp",
    "Top of ramp",
/*    "Sliding ticket",
    "Behind sliding ticket",
    "Tray deposit",
    "Jukebox light",
    "Behind jukebox light",
    "Red jukebox button",
    "Blue jukebox button",
    "Behind lever 1",
    "Behind lever 2",
    "Behind lever 3",*/
};

static constexpr size_t nprompts = sizeof(prompts)/sizeof(prompts[0]);

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