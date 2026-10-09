#include "mavsdk.hpp"
#include "plugins/dyt/dyt.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <vector>

using namespace mavsdk;

TEST(DytSystem, UdpInputPortOffset)
{
    Mavsdk ground_station{Mavsdk::Configuration{ComponentType::GroundStation}};
    Mavsdk autopilot{Mavsdk::Configuration{ComponentType::Autopilot}};

    ASSERT_EQ(
        ground_station.add_any_connection("udpin://127.0.0.1:17532"),
        ConnectionResult::Success);
    ASSERT_EQ(
        autopilot.add_any_connection("udpout://127.0.0.1:17632"),
        ConnectionResult::Success);
    ASSERT_TRUE(ground_station.first_autopilot(5.0));
}

TEST(DytSystem, TelemetryAndTrackingAcknowledgement)
{
    Mavsdk ground_station{Mavsdk::Configuration{ComponentType::GroundStation}};
    Mavsdk autopilot{Mavsdk::Configuration{ComponentType::Autopilot}};

    ASSERT_EQ(
        ground_station.add_any_connection("tcpin://127.0.0.1:17530"),
        ConnectionResult::Success);
    ASSERT_EQ(
        autopilot.add_any_connection("tcpout://127.0.0.1:17530"),
        ConnectionResult::Success);

    const auto system = ground_station.first_autopilot(10.0);
    ASSERT_TRUE(system);
    Dyt client{*system};
    DytServer server{autopilot.server_component()};

    auto telemetry_promise = std::make_shared<std::promise<Dyt::Telemetry>>();
    auto telemetry_future = telemetry_promise->get_future();
    auto once = std::make_shared<std::once_flag>();
    const auto telemetry_handle = client.subscribe_telemetry(
        [telemetry_promise, once](const Dyt::Telemetry& telemetry) {
            std::call_once(*once, [&] { telemetry_promise->set_value(telemetry); });
        });
    ASSERT_TRUE(telemetry_handle.valid());

    Dyt::Telemetry sample{};
    sample.status1 = 0x44;
    sample.zoom_low = 0x34;
    sample.status3 = 0xa2;
    sample.target_offset_x = -20;
    sample.laser_range = 234;
    ASSERT_EQ(server.publish_telemetry(sample), MavlinkDirectServer::Result::Success);
    ASSERT_EQ(telemetry_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    const auto received = telemetry_future.get();
    EXPECT_EQ(received.status1, sample.status1);
    EXPECT_EQ(received.zoom_tenths(), 0x234);
    EXPECT_EQ(received.target_offset_x, -20);
    EXPECT_EQ(received.laser_range, 234);
    client.unsubscribe_telemetry(telemetry_handle);

    std::atomic<int> command_count{0};
    std::mutex commands_mutex;
    std::vector<bool> command_values;
    const auto command_handle = server.subscribe_tracking_command(
        [&server, &command_count, &commands_mutex, &command_values](
            const DytServer::TrackingCommand& command) {
            const auto count = ++command_count;
            {
                std::lock_guard lock(commands_mutex);
                command_values.push_back(command.enabled);
            }
            server.acknowledge_tracking(
                command, count == 3 ? Dyt::Result::Denied : Dyt::Result::Success);
        });
    ASSERT_TRUE(command_handle.valid());

    EXPECT_EQ(client.set_tracking(true), Dyt::Result::Success);
    EXPECT_EQ(client.set_tracking(false), Dyt::Result::Success);
    EXPECT_EQ(client.set_tracking(true), Dyt::Result::Denied);
    EXPECT_EQ(command_count.load(), 3);
    {
        std::lock_guard lock(commands_mutex);
        EXPECT_EQ(command_values, (std::vector<bool>{true, false, true}));
    }
    server.unsubscribe_tracking_command(command_handle);
}
