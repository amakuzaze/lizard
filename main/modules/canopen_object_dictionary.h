#pragma once

#include "can.h"
#include "module.h"
#include <cstdint>
#include <map>
#include <memory>

class CanOpenObjectDictionary;
using CanOpenObjectDictionary_ptr = std::shared_ptr<CanOpenObjectDictionary>;

class CanOpenObjectDictionary : public Module {
public:
    struct Entry {
        enum Type {
            TYPE_U8,
            TYPE_U16,
            TYPE_U32,
            TYPE_I32,
        };

        enum Access {
            ACCESS_READONLY,
            ACCESS_WRITEONLY,
            ACCESS_READWRITE,
        };

        Type type;
        Access access;
        uint16_t index;
        uint8_t sub_index;
    };

    std::map<std::string, Entry> entries;

    CanOpenObjectDictionary(const std::string name);
    void call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) override;

    /* Returns nullptr if no entry exists under 'name' */
    const Entry *find_entry(const std::string &name) const;
};
