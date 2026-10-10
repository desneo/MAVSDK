#include "plugins/dyt/dyt.hpp"

#include "dyt_xml.hpp"
#include "server_component.hpp"
#include "system.hpp"
#include <nlohmann/json.hpp>

#include <condition_variable>
#include <limits>
#include <mutex>

namespace mavsdk {

using json = nlohmann::json;

namespace {
template<typename T> bool read_integer(const json& fields, const char* name, T& value)
{
    const auto it = fields.find(name);
    if (it == fields.end() || !it->is_number_integer()) {
        return false;
    }
    const auto number = it->get<int64_t>();
    if (number < std::numeric_limits<T>::min() || number > std::numeric_limits<T>::max()) {
        return false;
    }
    value = static_cast<T>(number);
    return true;
}
} // namespace

struct Dyt::State {
    std::mutex command_mutex;
    std::mutex ack_mutex;
    std::condition_variable ack_cv;
    uint16_t next_sequence{0};
    uint16_t pending_sequence{0};
    uint32_t pending_system{0};
    bool pending_enabled{false};
    bool waiting{false};
    Result result{Result::Timeout};
};

Dyt::Dyt(std::shared_ptr<System> system) :
    _system(std::move(system)),
    _direct(_system),
    _state(std::make_shared<State>())
{
    _xml_loaded = _direct.load_custom_xml(dyt_xml) == MavlinkDirect::Result::Success;
    if (!_xml_loaded) {
        return;
    }

    const std::weak_ptr<State> weak_state = _state;
    _ack_handle = _direct.subscribe_message(
        "DYT_TRACKING_ACK", [weak_state](const MavlinkDirect::MavlinkMessage& message) {
            const auto state = weak_state.lock();
            if (!state) {
                return;
            }
            const auto fields = json::parse(message.fields_json, nullptr, false);
            uint16_t sequence{};
            uint8_t enabled{};
            uint8_t code{};
            if (!fields.is_object() || !read_integer(fields, "sequence", sequence) ||
                !read_integer(fields, "enabled", enabled) ||
                !read_integer(fields, "result", code) || enabled > 1) {
                return;
            }

            std::lock_guard lock(state->ack_mutex);
            if (!state->waiting || message.system_id != state->pending_system ||
                sequence != state->pending_sequence || (enabled != 0) != state->pending_enabled) {
                return;
            }

            switch (code) {
                case 0: state->result = Result::Success; break;
                case 1: state->result = Result::Denied; break;
                case 2: state->result = Result::Failed; break;
                case 3: state->result = Result::Unsupported; break;
                default: state->result = Result::ProtocolError; break;
            }
            state->waiting = false;
            state->ack_cv.notify_one();
        });
}

Dyt::~Dyt()
{
    if (_ack_handle.valid()) {
        _direct.unsubscribe_message(_ack_handle);
    }
}

Dyt::TelemetryHandle Dyt::subscribe_telemetry(const TelemetryCallback& callback)
{
    if (!_xml_loaded || !callback) {
        return {};
    }
    return _direct.subscribe_message(
        "DYT_GIMBAL_TELEMETRY", [callback](const MavlinkDirect::MavlinkMessage& message) {
            const auto fields = json::parse(message.fields_json, nullptr, false);
            if (!fields.is_object()) {
                return;
            }
            Telemetry telemetry{};
            if (!read_integer(fields, "status1", telemetry.status1) ||
                !read_integer(fields, "status2", telemetry.status2) ||
                !read_integer(fields, "zoom_low", telemetry.zoom_low) ||
                !read_integer(fields, "status3", telemetry.status3) ||
                !read_integer(fields, "target_offset_x", telemetry.target_offset_x) ||
                !read_integer(fields, "target_offset_y", telemetry.target_offset_y) ||
                !read_integer(fields, "roll_angle", telemetry.roll_angle) ||
                !read_integer(fields, "pitch_angle", telemetry.pitch_angle) ||
                !read_integer(fields, "yaw_angle", telemetry.yaw_angle) ||
                !read_integer(fields, "reserved_16_17", telemetry.reserved_16_17) ||
                !read_integer(fields, "reserved_18_19", telemetry.reserved_18_19) ||
                !read_integer(fields, "roll_rate", telemetry.roll_rate) ||
                !read_integer(fields, "pitch_rate", telemetry.pitch_rate) ||
                !read_integer(fields, "yaw_rate", telemetry.yaw_rate) ||
                !read_integer(fields, "laser_range", telemetry.laser_range) ||
                !read_integer(fields, "self_test", telemetry.self_test) ||
                !read_integer(fields, "reserved_29_30", telemetry.reserved_29_30)) {
                return;
            }
            callback(telemetry);
        });
}

void Dyt::unsubscribe_telemetry(TelemetryHandle handle)
{
    if (handle.valid()) {
        _direct.unsubscribe_message(handle);
    }
}

Dyt::Result Dyt::set_tracking(bool enabled, std::chrono::milliseconds timeout)
{
    if (!_xml_loaded || !_ack_handle.valid() || timeout <= std::chrono::milliseconds::zero()) {
        return Result::ProtocolError;
    }
    if (!_system || !_system->is_connected() || _system->get_system_id() == 0) {
        return Result::NoSystem;
    }

    std::lock_guard command_lock(_state->command_mutex);
    const auto system_id = _system->get_system_id();
    uint16_t sequence{};
    {
        std::lock_guard ack_lock(_state->ack_mutex);
        sequence = ++_state->next_sequence;
        _state->pending_sequence = sequence;
        _state->pending_system = system_id;
        _state->pending_enabled = enabled;
        _state->waiting = true;
        _state->result = Result::Timeout;
    }

    MavlinkDirect::MavlinkMessage message{};
    message.message_name = "DYT_TRACKING_COMMAND";
    message.target_system_id = system_id;
    message.fields_json = json{
        {"sequence", sequence},
        {"target_system", system_id},
        {"target_component", 0},
        {"enabled", enabled ? 1 : 0},
    }.dump();

    const auto send_result = _direct.send_message(message);
    if (send_result != MavlinkDirect::Result::Success) {
        std::lock_guard ack_lock(_state->ack_mutex);
        _state->waiting = false;
        return Result::ProtocolError;
    }

    std::unique_lock ack_lock(_state->ack_mutex);
    if (!_state->ack_cv.wait_for(ack_lock, timeout, [this] { return !_state->waiting; })) {
        _state->waiting = false;
        return Result::Timeout;
    }
    return _state->result;
}

DytServer::DytServer(std::shared_ptr<ServerComponent> component) :
    _component(std::move(component)),
    _direct(_component)
{
    _xml_loaded = _direct.load_custom_xml(dyt_xml) == MavlinkDirectServer::Result::Success;
}

MavlinkDirectServer::Result DytServer::publish_telemetry(const Dyt::Telemetry& telemetry) const
{
    if (!_xml_loaded) {
        return MavlinkDirectServer::Result::Error;
    }
    MavlinkDirectServer::MavlinkMessage message{};
    message.message_name = "DYT_GIMBAL_TELEMETRY";
    message.fields_json = json{
        {"status1", telemetry.status1},
        {"status2", telemetry.status2},
        {"zoom_low", telemetry.zoom_low},
        {"status3", telemetry.status3},
        {"target_offset_x", telemetry.target_offset_x},
        {"target_offset_y", telemetry.target_offset_y},
        {"roll_angle", telemetry.roll_angle},
        {"pitch_angle", telemetry.pitch_angle},
        {"yaw_angle", telemetry.yaw_angle},
        {"reserved_16_17", telemetry.reserved_16_17},
        {"reserved_18_19", telemetry.reserved_18_19},
        {"roll_rate", telemetry.roll_rate},
        {"pitch_rate", telemetry.pitch_rate},
        {"yaw_rate", telemetry.yaw_rate},
        {"laser_range", telemetry.laser_range},
        {"self_test", telemetry.self_test},
        {"reserved_29_30", telemetry.reserved_29_30},
    }.dump();
    return _direct.send_message(message);
}

DytServer::TrackingCommandHandle
DytServer::subscribe_tracking_command(const TrackingCommandCallback& callback)
{
    if (!_xml_loaded || !callback) {
        return {};
    }
    const auto component_id = _component->component_id();
    return _direct.subscribe_message(
        "DYT_TRACKING_COMMAND",
        [callback, component_id](const MavlinkDirectServer::MavlinkMessage& message) {
            const auto fields = json::parse(message.fields_json, nullptr, false);
            if (!fields.is_object() || message.system_id == 0 || message.system_id > 255 ||
                message.component_id > 255) {
                return;
            }
            TrackingCommand command{};
            command.source_system = message.system_id;
            command.source_component = message.component_id;
            uint8_t target_component{};
            uint8_t enabled{};
            if (!read_integer(fields, "sequence", command.sequence) ||
                !read_integer(fields, "target_component", target_component) ||
                !read_integer(fields, "enabled", enabled) || enabled > 1 ||
                (target_component != 0 && target_component != component_id)) {
                return;
            }
            command.enabled = enabled != 0;
            callback(command);
        });
}

void DytServer::unsubscribe_tracking_command(TrackingCommandHandle handle)
{
    if (handle.valid()) {
        _direct.unsubscribe_message(handle);
    }
}

MavlinkDirectServer::Result
DytServer::acknowledge_tracking(const TrackingCommand& command, Dyt::Result result) const
{
    if (!_xml_loaded || command.source_system == 0 || command.source_system > 255 ||
        command.source_component > 255) {
        return MavlinkDirectServer::Result::InvalidField;
    }
    uint8_t code{};
    switch (result) {
        case Dyt::Result::Success: code = 0; break;
        case Dyt::Result::Denied: code = 1; break;
        case Dyt::Result::Failed: code = 2; break;
        case Dyt::Result::Unsupported: code = 3; break;
        default: return MavlinkDirectServer::Result::InvalidField;
    }
    MavlinkDirectServer::MavlinkMessage message{};
    message.message_name = "DYT_TRACKING_ACK";
    message.target_system_id = command.source_system;
    message.target_component_id = command.source_component;
    message.fields_json = json{
        {"sequence", command.sequence},
        {"target_system", command.source_system},
        {"target_component", command.source_component},
        {"enabled", command.enabled ? 1 : 0},
        {"result", code},
    }.dump();
    return _direct.send_message(message);
}

} // namespace mavsdk
