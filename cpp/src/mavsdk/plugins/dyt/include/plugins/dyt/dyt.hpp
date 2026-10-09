#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include "plugin_base.hpp"
#include "plugins/mavlink_direct/mavlink_direct.hpp"
#include "plugins/mavlink_direct_server/mavlink_direct_server.hpp"
#include "server_plugin_base.hpp"

namespace mavsdk {

class System;
class ServerComponent;

/** Typed MAVSDK interface for the DYT gimbal MAVLink messages. */
class MAVSDK_PUBLIC Dyt final : public PluginBase {
public:
    /** Values are the raw DYT serial frame fields, before applying the documented scale. */
    struct Telemetry {
        uint8_t status1{};
        uint8_t status2{};
        uint8_t zoom_low{};
        uint8_t status3{};
        int16_t target_offset_x{}; // 0.05 degree
        int16_t target_offset_y{}; // 0.05 degree
        int16_t roll_angle{}; // 0.01 degree
        int16_t pitch_angle{}; // 0.01 degree
        int16_t yaw_angle{}; // 0.01 degree
        uint16_t reserved_16_17{};
        uint16_t reserved_18_19{};
        int16_t roll_rate{}; // 0.01 degree/s
        int16_t pitch_rate{}; // 0.01 degree/s
        int16_t yaw_rate{}; // 0.01 degree/s
        uint16_t laser_range{}; // 0.1 metre; zero is invalid
        uint8_t self_test{};
        uint16_t reserved_29_30{};

        uint16_t zoom_tenths() const
        {
            return static_cast<uint16_t>(zoom_low | ((status3 & 0x0f) << 8));
        }
    };

    enum class Result {
        Success,
        Denied,
        Failed,
        Unsupported,
        Timeout,
        NoSystem,
        ProtocolError,
    };

    using TelemetryCallback = std::function<void(const Telemetry&)>;
    using TelemetryHandle = MavlinkDirect::MessageHandle;

    explicit Dyt(std::shared_ptr<System> system);
    ~Dyt() override;

    /** Subscribe to DYT_GIMBAL_TELEMETRY; malformed messages are ignored. */
    TelemetryHandle subscribe_telemetry(const TelemetryCallback& callback);
    void unsubscribe_telemetry(TelemetryHandle handle);

    /** Send the DYT tracking switch and wait for a matching application ACK.
     * Do not call from a MAVSDK callback, since callbacks deliver the ACK.
     * Only one call per Dyt instance runs at a time.
     */
    Result set_tracking(bool enabled, std::chrono::milliseconds timeout = std::chrono::seconds(3));

    Dyt(const Dyt&) = delete;
    Dyt& operator=(const Dyt&) = delete;

private:
    struct State;
    std::shared_ptr<System> _system;
    MavlinkDirect _direct;
    std::shared_ptr<State> _state;
    MavlinkDirect::MessageHandle _ack_handle;
    bool _xml_loaded{false};
};

/** Controller-side interface for publishing DYT telemetry and acknowledging tracking commands. */
class MAVSDK_PUBLIC DytServer final : public ServerPluginBase {
public:
    struct TrackingCommand {
        uint16_t sequence{};
        uint32_t source_system{};
        uint32_t source_component{};
        bool enabled{false};
    };

    using TrackingCommandCallback = std::function<void(const TrackingCommand&)>;
    using TrackingCommandHandle = MavlinkDirectServer::MessageHandle;

    explicit DytServer(std::shared_ptr<ServerComponent> component);
    ~DytServer() override = default;

    /** Publish one 60 Hz sample. Scheduling at 60 Hz is the caller's responsibility. */
    MavlinkDirectServer::Result publish_telemetry(const Dyt::Telemetry& telemetry) const;

    /** The callback must call acknowledge_tracking after executing the requested switch. */
    TrackingCommandHandle subscribe_tracking_command(const TrackingCommandCallback& callback);
    void unsubscribe_tracking_command(TrackingCommandHandle handle);

    MavlinkDirectServer::Result
    acknowledge_tracking(const TrackingCommand& command, Dyt::Result result) const;

    DytServer(const DytServer&) = delete;
    DytServer& operator=(const DytServer&) = delete;

private:
    std::shared_ptr<ServerComponent> _component;
    MavlinkDirectServer _direct;
    bool _xml_loaded{false};
};

} // namespace mavsdk
