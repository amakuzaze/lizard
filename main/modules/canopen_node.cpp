#include "canopen_node.h"
#include "global.h"
#include "uart.h"
#include <cinttypes>
#include <esp_timer.h>
#include <map>
#include <stdexcept>
#include <string_view>

using namespace std::string_view_literals;

enum CobFunction {
    COB_SYNC_EMCY = 0x1,
    COB_TPDO1 = 0x3,
    COB_RPDO1 = 0x4,
    COB_TPDO2 = 0x5,
    COB_RPDO2 = 0x6,
    COB_TPDO3 = 0x7,
    COB_RPDO3 = 0x8,
    COB_TPDO4 = 0x9,
    COB_RPDO4 = 0xA,
    COB_SDO_SERVER2CLIENT = 0xB,
    COB_SDO_CLIENT2SERVER = 0xC,
    COB_HEARTBEAT = 0xE,
};

enum NmtStateChange : std::uint8_t {
    STATE_CHANGE_OPERATIONAL = 0x1,
    STATE_CHANGE_PREOPERATIONAL = 0x80,
    STATE_CHANGE_RESET_NODE = 0x81,
    STATE_CHANGE_RESET_COM = 0x82,
};

static const std::map<std::string_view, NmtStateChange> nmt_state_change_map = {
    {"set_operational"sv, STATE_CHANGE_OPERATIONAL},
    {"set_preoperational"sv, STATE_CHANGE_PREOPERATIONAL},
    {"reset_node"sv, STATE_CHANGE_RESET_NODE},
    {"reset_com"sv, STATE_CHANGE_RESET_COM},
};

static uint32_t build_cob_id(CobFunction cmd, uint8_t node_id) {
    return (cmd << (11 - 4)) | (node_id);
}

static void unwrap_cob_id(uint32_t cob_id, uint8_t &function_out, uint8_t &node_id_out) {
    function_out = (cob_id >> (11 - 4)) & 0xF;
    node_id_out = cob_id & 0x7F;
}

static void build_write_od_value_8u(uint16_t idx, uint8_t sub, uint8_t val, uint8_t *d) {
    d[0] = (0x1 /*write*/ << (8 - 3)) | (3 << 2 /*8bit*/) | (1 << 1 /*expedited*/) | (1 /*size indicated*/);
    d[1] = idx & 0xFF; // idx low
    d[2] = idx >> 8;   // idx high
    d[3] = sub;        // idx sub
    d[4] = val;
}

static void build_write_od_value_16u(uint16_t idx, uint8_t sub, uint16_t val, uint8_t *d) {
    d[0] = (0x1 /*write*/ << (8 - 3)) | (2 << 2 /*16bit*/) | (1 << 1 /*expedited*/) | (1 /*size indicated*/);
    d[1] = idx & 0xFF; // idx low
    d[2] = idx >> 8;   // idx high
    d[3] = sub;        // idx sub
    d[4] = val & 0xFF;
    d[5] = (val >> 8) & 0xFF;
}

static void build_write_od_value_32u(uint16_t idx, uint8_t sub, uint32_t val, uint8_t *d) {
    d[0] = (0x1 /*write*/ << (8 - 3)) | (1 << 1 /*expedited*/) | (1 /*size indicated*/);
    d[1] = idx & 0xFF; // idx low
    d[2] = idx >> 8;   // idx high
    d[3] = sub;        // idx sub
    d[4] = val & 0xFF;
    d[5] = (val >> 8) & 0xFF;
    d[6] = (val >> 16) & 0xFF;
    d[7] = (val >> 24) & 0xFF;
}

static const std::string ATTR_LAST_HEARTBEAT{"last_heartbeat"};

CanOpenNode::CanOpenNode(const std::string name, Can_ptr can, int node_id)
    : Module(canopen_node, name), can{can}, current_state{0} {
    if (node_id < 1 || node_id > 127) {
        throw std::runtime_error{"CanOpenNode node_id not in valid range"};
    }

    this->node_id = static_cast<uint8_t>(node_id);
    this->properties[ATTR_LAST_HEARTBEAT] = std::make_shared<IntegerVariable>();
}

void CanOpenNode::call(const std::string method_name, const std::vector<ConstExpression_ptr> arguments) {
    constexpr std::string_view nmt_prefix{"nmt_"sv};
    const std::size_t nmt_found = method_name.find(nmt_prefix);

    if (nmt_found != std::string::npos) {
        std::string_view state{&method_name[nmt_prefix.size()], method_name.size() - nmt_prefix.size()};
        const auto iter = nmt_state_change_map.find(state);

        if (iter != nmt_state_change_map.end()) {
            NmtStateChange state_change = iter->second;
            const uint8_t data[]{state_change, this->node_id};
            this->can->send(0, data, false, 2);
        } else {
            echo("unknown nmt state change: [%.*s]", state.length(), state.data());
        }

        return;
    }

    if (method_name == "last_heartbeat"sv) {
        Module::expect(arguments, 0);
        echo("Last heartbeat for node [%02X]: %" PRId64 " ago",
             node_id, esp_timer_get_time() - this->properties[ATTR_LAST_HEARTBEAT]->integer_value);
    } else if (method_name == "set_dictionary"sv) {
        expect(arguments, 1, identifier);
        std::string module_name = arguments[0]->evaluate_identifier();
        Module_ptr module = Global::get_module(module_name);
        if (module->type != canopen_object_dictionary) {
            throw std::runtime_error("module \"" + module_name + "\" is no object dictionary");
        }
        this->obj_dict = std::static_pointer_cast<CanOpenObjectDictionary>(module);
    } else if (method_name == "sdo_write_integer") {
        if (this->obj_dict == nullptr) {
            throw std::runtime_error("no object dictionary set");
        }

        expect(arguments, 2, string, integer);

        const std::string entry_name = arguments[0]->evaluate_string();
        const CanOpenObjectDictionary::Entry *entry = this->obj_dict->find_entry(entry_name);
        if (entry == nullptr) {
            throw std::runtime_error("no object dictionary entry for " + entry_name);
        }

        if (entry->access == CanOpenObjectDictionary::Entry::ACCESS_READONLY) {
            throw std::runtime_error(entry_name + " is read-only");
        }

        int64_t value_arg = arguments[1]->evaluate_integer();
        uint8_t data[8];
        uint32_t cob_id = build_cob_id(COB_SDO_CLIENT2SERVER, this->node_id);

        switch (entry->type) {
        case CanOpenObjectDictionary::Entry::TYPE_U8:
            if (value_arg < 0 || value_arg > UINT8_MAX) {
                echo("Value '%" PRIX64 "' out of range, must fit into unsigned 8 bit", value_arg);
                return;
            }

            build_write_od_value_8u(entry->index, entry->sub_index, static_cast<uint8_t>(value_arg), data);
            break;

        case CanOpenObjectDictionary::Entry::TYPE_U16:
            if (value_arg < 0 || value_arg > UINT16_MAX) {
                echo("Value '%" PRIX64 "' out of range, must fit into unsigned 16 bit", value_arg);
                return;
            }

            build_write_od_value_16u(entry->index, entry->sub_index, static_cast<uint16_t>(value_arg), data);
            break;

        case CanOpenObjectDictionary::Entry::TYPE_U32:
            if (value_arg < 0 || value_arg > UINT32_MAX) {
                echo("Value '%" PRIX64 "' out of range, must fit into unsigned 32 bit", value_arg);
                return;
            }

            build_write_od_value_32u(entry->index, entry->sub_index, static_cast<uint32_t>(value_arg), data);
            break;

        case CanOpenObjectDictionary::Entry::TYPE_I32:
            if (value_arg < INT32_MIN || value_arg > INT32_MAX) {
                echo("Value '%" PRIX64 "' out of range, must fit into signed 32 bit", value_arg);
                return;
            }

            build_write_od_value_32u(entry->index, entry->sub_index, static_cast<uint32_t>(value_arg), data);
            break;
        }

        this->can->send(cob_id, data);
    } else {
        Module::call(method_name, arguments);
    }
}

void CanOpenNode::handle_can_msg(const uint32_t id, const int count, const uint8_t *data) {
    uint8_t function, msg_node_id;
    unwrap_cob_id(id, function, msg_node_id);

    if (msg_node_id != this->node_id) {
        echo("Message for invalid node id received: %d (should be %d)");
        return;
    }

    switch (function) {
    case COB_HEARTBEAT:
        this->properties[ATTR_LAST_HEARTBEAT]->integer_value = esp_timer_get_time();
        current_state = data[0];
        echo("(%" PRId64 ") Node [%s] heartbeat", esp_timer_get_time(), this->name.c_str());
        break;
    case COB_SDO_SERVER2CLIENT: {
        uint8_t scs = data[0] >> (8 - 3);
        uint16_t index = data[1] | data[2] << 8;
        uint8_t sub_index = data[3];
        uint32_t value = data[4] | data[5] << 8 | data[6] << 16 | data[7] << 24;

        switch (scs) {
        case 2:
            // TODO: Properly communicate the read value upward
            echo("Read [%s]: [%04Xh.%02Xh]: 0x%08X (%d)",
                 this->name.c_str(), index, sub_index, value, *((int32_t *)&value));
            break;

        case 3:
            echo("Write [%s]: [%04Xh.%02Xh] successful", this->name.c_str(), index, sub_index);
            break;

        case 4:
            switch (value) {
            case 0x06020000:
                echo("[%04Xh.%02Xh] does not exist", index, sub_index);
                break;

            case 0x06070010:
                echo("[%04Xh.%02Xh]: type mismatch", index, sub_index);
                break;
            }

            break;
        }

        break;
    }

    default:
        echo("Node %02X: unhandled function %X", node_id, function);
    }
}

void CanOpenNode::subscribe_to_can() {
    can->subscribe(build_cob_id(COB_HEARTBEAT, node_id), this->shared_from_this());
    can->subscribe(build_cob_id(COB_SDO_SERVER2CLIENT, node_id), this->shared_from_this());
}
