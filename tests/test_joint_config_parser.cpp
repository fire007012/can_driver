#include <gtest/gtest.h>

#include "can_driver/JointConfigParser.h"

#include <string>
#include <vector>

using joint_config_parser::parse;
using joint_config_parser::parseMotorId;

namespace {

// 构造最小合法关节配置，供各类异常分支复用。
XmlRpc::XmlRpcValue makeJointBase(const std::string &name,
                                  const XmlRpc::XmlRpcValue &motorId,
                                  const std::string &protocol = "MT")
{
    XmlRpc::XmlRpcValue joint;
    joint["name"] = name;
    joint["motor_id"] = motorId;
    joint["protocol"] = protocol;
    joint["can_device"] = std::string("can0");
    joint["control_mode"] = std::string("position");
    return joint;
}

XmlRpc::XmlRpcValue makeJointList(const XmlRpc::XmlRpcValue &joint0)
{
    XmlRpc::XmlRpcValue list;
    list.setSize(1);
    list[0] = joint0;
    return list;
}

} // namespace

TEST(JointConfigParser, ValidIntMotorId)
{
    XmlRpc::XmlRpcValue v(42);
    MotorID out;
    std::string err;
    EXPECT_TRUE(parseMotorId(v, "test", out, err));
    EXPECT_EQ(static_cast<uint16_t>(out), 42);
}

TEST(JointConfigParser, ValidHexStringMotorId)
{
    XmlRpc::XmlRpcValue v(std::string("0x1A"));
    MotorID out;
    std::string err;
    EXPECT_TRUE(parseMotorId(v, "test", out, err));
    EXPECT_EQ(static_cast<uint16_t>(out), 0x1A);
}

TEST(JointConfigParser, NegativeMotorIdFails)
{
    XmlRpc::XmlRpcValue v(-1);
    MotorID out;
    std::string err;
    EXPECT_FALSE(parseMotorId(v, "test", out, err));
    EXPECT_FALSE(err.empty());
}

TEST(JointConfigParser, OverflowMotorIdFails)
{
    XmlRpc::XmlRpcValue v(70000);
    MotorID out;
    std::string err;
    EXPECT_FALSE(parseMotorId(v, "test", out, err));
}

TEST(JointConfigParser, ParseRejectsNonArray)
{
    XmlRpc::XmlRpcValue notArray(42);
    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(notArray, out, err));
    EXPECT_NE(err.find("non-empty list"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsEmptyArray)
{
    XmlRpc::XmlRpcValue emptyList;
    emptyList.setSize(0);
    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(emptyList, out, err));
    EXPECT_NE(err.find("non-empty list"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsMissingRequiredField)
{
    XmlRpc::XmlRpcValue joint;
    joint["name"] = std::string("j0");
    joint["motor_id"] = 1;
    joint["protocol"] = std::string("MT");
    joint["can_device"] = std::string("can0");
    // missing control_mode

    auto list = makeJointList(joint);
    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(list, out, err));
    EXPECT_NE(err.find("missing required field"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsInvalidProtocol)
{
    XmlRpc::XmlRpcValue motorId(1);
    auto joint = makeJointBase("j0", motorId, "UNKNOWN");
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(list, out, err));
    EXPECT_NE(err.find("unknown protocol"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsInvalidControlMode)
{
    XmlRpc::XmlRpcValue motorId(1);
    auto joint = makeJointBase("j0", motorId);
    joint["control_mode"] = std::string("velcity");
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(list, out, err));
    EXPECT_NE(err.find("unknown control_mode"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsInvalidPositionScale)
{
    XmlRpc::XmlRpcValue motorId(1);
    auto joint = makeJointBase("j0", motorId);
    joint["position_scale"] = 0.0;
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(list, out, err));
    EXPECT_NE(err.find("invalid position_scale"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsInvalidVelocityScale)
{
    XmlRpc::XmlRpcValue motorId(1);
    auto joint = makeJointBase("j0", motorId);
    joint["velocity_scale"] = -0.1;
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(list, out, err));
    EXPECT_NE(err.find("invalid velocity_scale"), std::string::npos);
}

TEST(JointConfigParser, ParseRejectsInvalidMotorIdInMainParser)
{
    XmlRpc::XmlRpcValue motorId(std::string("not_a_number"));
    auto joint = makeJointBase("j0", motorId);
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    EXPECT_FALSE(parse(list, out, err));
    EXPECT_NE(err.find("invalid motor_id"), std::string::npos);
}

TEST(JointConfigParser, ParseUsesDefaultScalesWhenNotProvided)
{
    XmlRpc::XmlRpcValue motorId(7);
    auto joint = makeJointBase("joint_default_scale", motorId, "PP");
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    ASSERT_TRUE(parse(list, out, err));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].name, "joint_default_scale");
    EXPECT_EQ(out[0].canDevice, "can0");
    EXPECT_EQ(out[0].controlMode, "position");
    EXPECT_EQ(out[0].protocol, CanType::PP);
    EXPECT_EQ(static_cast<uint16_t>(out[0].motorId), 7u);
    EXPECT_DOUBLE_EQ(out[0].positionScale, 1.0);
    EXPECT_DOUBLE_EQ(out[0].velocityScale, 1.0);
}

TEST(JointConfigParser, ParseReadsExplicitScales)
{
    XmlRpc::XmlRpcValue motorId(8);
    auto joint = makeJointBase("joint_explicit_scale", motorId);
    joint["position_scale"] = 0.01;
    joint["velocity_scale"] = 0.02;
    auto list = makeJointList(joint);

    std::vector<joint_config_parser::ParsedJointConfig> out;
    std::string err;
    ASSERT_TRUE(parse(list, out, err));
    ASSERT_EQ(out.size(), 1u);
    EXPECT_DOUBLE_EQ(out[0].positionScale, 0.01);
    EXPECT_DOUBLE_EQ(out[0].velocityScale, 0.02);
}

TEST(JointConfigParser, InvalidStringMotorIdFails)
{
    XmlRpc::XmlRpcValue v(std::string("not_a_number"));
    MotorID out;
    std::string err;
    EXPECT_FALSE(parseMotorId(v, "test", out, err));
}
