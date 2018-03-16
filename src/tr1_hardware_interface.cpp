#include <sstream>
#include <tr1_hardware_interface/tr1_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <tr1cpp/tr1.h>
#include <tr1cpp/joint.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace tr1_hardware_interface
{
	TR1HardwareInterface::TR1HardwareInterface(ros::NodeHandle& nh) \
		: nh_(nh)
	{
		init();
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

		nh_.param("/tr1/hardware_interface/loop_hz", loop_hz_, 0.1);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &TR1HardwareInterface::update, this);

		ROS_INFO_NAMED("hardware_interface", "Loaded generic_hardware_interface.");
	}

	TR1HardwareInterface::~TR1HardwareInterface()
	{
	}

	void TR1HardwareInterface::init()
	{
		//joint_mode_ = 3; // ONLY EFFORT FOR NOW
		// Get joint names
		nh_.getParam("/tr1/hardware_interface/joints", joint_names_);
		if (joint_names_.size() == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}
		num_joints_ = joint_names_.size();

		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		joint_effort_.resize(num_joints_);
		joint_position_command_.resize(num_joints_);
		joint_velocity_command_.resize(num_joints_);
		joint_effort_command_.resize(num_joints_);


		// Initialize controller
		for (int i = 0; i < num_joints_; ++i)
		{
		  ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

			tr1.armRight.joints[i].name = joint_names_[i];
			nh_.getParam("/tr1/joint_offsets/" + joint_names_[i], tr1.armRight.joints[i].angleOffset);
			nh_.getParam("/tr1/joint_read_ratio/" + joint_names_[i], tr1.armRight.joints[i].readRatio);

		  // Create joint state interface
			JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		  joint_state_interface_.registerHandle(jointStateHandle);

		  // Create position joint interface
			JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
			JointLimits limits;
 	   	SoftJointLimits softLimits;
			if (getJointLimits(joint_names_[i], nh_, limits) == false) {
				ROS_ERROR_STREAM("Cannot set joint limits for " << joint_names_[i]);
			} else {
				PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
				positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
			}
		  position_joint_interface_.registerHandle(jointPositionHandle);

		  // Create velocity joint interface
			//JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
		  //effort_joint_interface_.registerHandle(jointVelocityHandle);

		  // Create effort joint interface
			JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
		  effort_joint_interface_.registerHandle(jointEffortHandle);

		}

		registerInterface(&joint_state_interface_);
		registerInterface(&position_joint_interface_);
		registerInterface(&velocity_joint_interface_);
		registerInterface(&effort_joint_interface_);
		registerInterface(&positionJointSoftLimitsInterface);
	}

	void TR1HardwareInterface::update(const ros::TimerEvent& e)
	{
		_logInfo = "\n";
		_logInfo += "Joint Position Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointPositionStr;
			jointPositionStr << joint_position_command_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
		}

		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

		read();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write(elapsed_time_);

		ROS_INFO_STREAM(_logInfo);
	}

	void TR1HardwareInterface::read()
	{
		_logInfo += "Joint State:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			if (tr1.armRight.joints[i].getActuatorType() == ACTUATOR_TYPE_MOTOR)
			{
				joint_position_[i] = tr1.armRight.joints[i].readAngle();

				std::ostringstream jointPositionStr;
				jointPositionStr << joint_position_[i];
				_logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
			}
		}
	}

	void TR1HardwareInterface::write(ros::Duration elapsed_time)
	{
		positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

		_logInfo += "Joint Effort Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			double jointEffortMax = 1.0;
			double jointEffortMin = -1.0;

			nh_.getParam("/tr1/joint_limits/" + joint_names_[i] + "/max_effort", jointEffortMax);
			nh_.getParam("/tr1/joint_limits/" + joint_names_[i] + "/min_effort", jointEffortMin);

			if (joint_effort_command_[i] > jointEffortMax) joint_effort_command_[i] = jointEffortMax;
			if (joint_effort_command_[i] < jointEffortMin) joint_effort_command_[i] = jointEffortMin;

			tr1.armRight.joints[i].actuate(joint_effort_command_[i]);

			std::ostringstream jointEffortStr;
			jointEffortStr << joint_effort_command_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointEffortStr.str() + "\n";
		}
	}
}
