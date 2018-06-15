#include <sstream>
#include <test_hardware_interface/test_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <ROBOTcpp/ROBOT.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace test_hardware_interface
{
    testHardwareInterface::testHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
	init();
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_))l
	nh_.param("/test/hardware_interface/loop_hz", loop_hz_, 0.1);
	ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	non_realtime_loop_ = nh_.createTimer(update_freq, &TR1HardwareInterface::update, this);
    }

    testHardwareInterface::~testHardwareInterface() {

    }

    void testHardwareInterface::init() {
