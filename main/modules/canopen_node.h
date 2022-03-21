#pragma once

#include "can.h"
#include "canopen_object_dictionary.h"
#include "module.h"
#include <memory>

class CanOpenNode;
using CanOpenNode_ptr = std::shared_ptr<CanOpenNode>;

class CanOpenNode : public Module, public std::enable_shared_from_this<CanOpenNode> {
private:
    Can_ptr can;
    uint8_t node_id;
    uint8_t current_state;
    CanOpenObjectDictionary_ptr obj_dict;

public:
    CanOpenNode(const std::string name, Can_ptr can, int node_id);
    void call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) override;
    void handle_can_msg(const uint32_t id, const int count, const uint8_t *const data) override;

    void subscribe_to_can();
};
