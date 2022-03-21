#include "canopen_object_dictionary.h"
#include "uart.h"
#include <cinttypes>

CanOpenObjectDictionary::CanOpenObjectDictionary(const std::string name)
    : Module(canopen_object_dictionary, name) {
}

void CanOpenObjectDictionary::call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) {
    if (method_name == "add") {
        expect(arguments, 5, string, string, string, integer, integer);
        std::string name = arguments[0]->evaluate_string();
        if (entries.find(name) != entries.end()) {
            echo("Entry '%s' already exists!", name.c_str());
            return;
        }

        Entry entry{};

        std::string type_arg = arguments[1]->evaluate_string();
        if (type_arg == "u8") {
            entry.type = Entry::TYPE_U8;
        } else if (type_arg == "u16") {
            entry.type = Entry::TYPE_U16;
        } else if (type_arg == "u32") {
            entry.type = Entry::TYPE_U32;
        } else if (type_arg == "i32") {
            entry.type = Entry::TYPE_I32;
        } else {
            echo("Invaild size '%d', must be one of u8, u16, u32, i32", type_arg);
        }

        std::string access_arg = arguments[2]->evaluate_string();
        if (access_arg == "ro") {
            entry.access = Entry::ACCESS_READONLY;
        } else if (access_arg == "wo") {
            entry.access = Entry::ACCESS_WRITEONLY;
        } else if (access_arg == "rw") {
            entry.access = Entry::ACCESS_READWRITE;
        } else {
            echo("Invalid access '%s', must be one of 'ro', 'wo', 'rw'", access_arg.c_str());
            return;
        }

        int64_t index_arg = arguments[3]->evaluate_integer();
        if (index_arg < 0 || index_arg > UINT16_MAX) {
            echo("Index '%" PRIX64 "' out of range, must fit into unsigned 16 bit", index_arg);
            return;
        }

        int64_t sub_index_arg = arguments[4]->evaluate_integer();
        if (sub_index_arg < 0 || sub_index_arg > UINT8_MAX) {
            echo("Sub index '%" PRIX64 "' out of range, must fit into unsigned 8 bit", sub_index_arg);
            return;
        }

        entry.index = static_cast<uint16_t>(index_arg);
        entry.sub_index = static_cast<uint8_t>(sub_index_arg);

        entries.emplace(std::move(name), entry);

    } else {
        Module::call(method_name, arguments);
    }
}

const CanOpenObjectDictionary::Entry *CanOpenObjectDictionary::find_entry(const std::string &name) const {
    auto iter = entries.find(name);

    if (iter == entries.end()) {
        return nullptr;
    } else {
        return &iter->second;
    }
}
