#pragma once

#include "../compilation/expression.h"
#include "../compilation/variable.h"
#include <list>
#include <map>
#include <string>
#include <vector>

enum ModuleType {
    bluetooth,
    core,
    expander,
    input,
    output,
    can,
    serial,
    odrive_motor,
    odrive_wheels,
    rmd_motor,
    roboclaw,
    roboclaw_motor,
    proxy,
    canopen_node,
    canopen_object_dictionary,
    number_of_module_types,
};

class Module;
using Module_ptr = std::shared_ptr<Module>;
using ConstModule_ptr = std::shared_ptr<const Module>;

class Module {
private:
    std::list<Module_ptr> shadow_modules;

protected:
    std::map<std::string, Variable_ptr> properties;
    bool output_on = false;
    bool broadcast = false;

public:
    const ModuleType type;
    const std::string name;

    Module(const ModuleType type, const std::string name);
    static void expect(const std::vector<ConstExpression_ptr> arguments, const int num, ...);
    static Module_ptr create(const std::string type,
                             const std::string name,
                             const std::vector<ConstExpression_ptr> arguments,
                             void (*message_handler)(const char *));
    virtual void step();
    virtual void call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments);
    void call_with_shadows(const std::string method_name, const std::vector<ConstExpression_ptr> arguments);
    virtual std::string get_output() const;
    Variable_ptr get_property(const std::string property_name) const;
    virtual void write_property(const std::string property_name, const ConstExpression_ptr expression);
    virtual void handle_can_msg(const uint32_t id, const int count, const uint8_t *const data);
};
