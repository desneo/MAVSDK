# DYT MAVLink interface

`dyt.xml` is the source of truth for the three MAVLink 2 messages:

| ID | Message | Direction |
| --- | --- | --- |
| 53000 | `DYT_GIMBAL_TELEMETRY` | Controller to client, normally 60 Hz |
| 53001 | `DYT_TRACKING_COMMAND` | Client to controller |
| 53002 | `DYT_TRACKING_ACK` | Controller to client after execution |

The telemetry fields map to bytes 2–30 of the 32-byte DYT serial telemetry frame in `agent_zsh/DYT控制协议.docx`. The serial sync bytes and serial checksum are not sent in MAVLink; MAVLink supplies its own framing and CRC. All numeric fields retain the integer scale from the DYT document. In particular, target offsets are 0.05° per count, angles and rates are 0.01° or 0.01°/s per count, range is 0.1 m per count, and `zoom_tenths()` combines the low byte and the low nibble of `status3`.

Both interfaces load `dyt.xml` into MAVSDK's message set automatically. Another MAVLink implementation must load the same XML before exchanging these messages.

```cpp
#include <mavsdk/plugins/dyt/dyt.hpp>

// Client: `system` is a discovered std::shared_ptr<mavsdk::System>.
mavsdk::Dyt dyt{system};
auto telemetry_handle = dyt.subscribe_telemetry([](const mavsdk::Dyt::Telemetry& sample) {
    // Read sample.status1, sample.zoom_tenths(), sample.roll_angle, etc.
});
auto start_result = dyt.set_tracking(true);
auto stop_result = dyt.set_tracking(false);
dyt.unsubscribe_telemetry(telemetry_handle);

// Controller: `component` comes from mavsdk.server_component().
mavsdk::DytServer server{component};
auto command_handle = server.subscribe_tracking_command(
    [&server](const mavsdk::DytServer::TrackingCommand& command) {
        // Apply command.enabled to the DYT device before acknowledging it.
        server.acknowledge_tracking(command, mavsdk::Dyt::Result::Success);
    });
mavsdk::Dyt::Telemetry sample{};
server.publish_telemetry(sample);
server.unsubscribe_tracking_command(command_handle);
```

`set_tracking` waits for a matching `DYT_TRACKING_ACK` with the same sequence and requested state. `Success` means the controller reported acceptance after execution. The default timeout is three seconds. The controller may instead reply `Denied`, `Failed`, or `Unsupported`. Call `set_tracking` from an application thread, since MAVSDK callback threads also deliver the ACK.
