#include "plugins/dyt/dyt.hpp"

#include <mav/MessageSet.h>
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace mavsdk {
namespace {

TEST(Dyt, XmlDefinesAllMessages)
{
    std::ifstream file("src/mavsdk/plugins/dyt/dyt.xml");
    ASSERT_TRUE(file.good());
    std::ostringstream xml_stream;
    xml_stream << file.rdbuf();
    const std::string xml = xml_stream.str();

    mav::MessageSet definitions;
    ASSERT_EQ(definitions.addFromXMLString(xml, false), mav::MessageSetResult::Success);
    EXPECT_EQ(definitions.create("DYT_GIMBAL_TELEMETRY")->id(), 53000);
    EXPECT_EQ(definitions.create("DYT_TRACKING_COMMAND")->id(), 53001);
    EXPECT_EQ(definitions.create("DYT_TRACKING_ACK")->id(), 53002);
}

TEST(Dyt, ZoomUsesLowByteAndStatus3LowNibble)
{
    Dyt::Telemetry telemetry{};
    telemetry.zoom_low = 0x34;
    telemetry.status3 = 0xa2;
    EXPECT_EQ(telemetry.zoom_tenths(), 0x234);
}

} // namespace
} // namespace mavsdk
