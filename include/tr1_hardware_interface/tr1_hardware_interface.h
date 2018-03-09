#ifndef ROS_CONTROL__TR1_HARDWARE_INTERFACE_H
#define ROS_CONTROL__TR1_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <tr1cpp/tr1.h>
#include <tr1cpp/arm.h>
#include <tr1cpp/joint.h>

// TR1 hardware base class
#include <tr1_hardware_interface/tr1_hardware.h>

namespace tr1_hardware_interface
{
	// For simulation only - determines how fast a trajectory is followed
	static const double POSITION_STEP_FACTOR = 10;
	static const double VELOCITY_STEP_FACTOR = 10;

	/// \brief Hardware interface for a robot
	class TR1HardwareInterface: public tr1_hardware_interface::TR1Hardware
	{
		public:
			/// \param nh  Node handle for topics.
			TR1HardwareInterface(ros::NodeHandle& nh);

			/// \brief Destructor.
			~TR1HardwareInterface();

			/// \brief Initialize the hardware interface
			void init();

			/// \brief Timer event
			void update(const ros::TimerEvent& e);

			/// \brief Read the state from the robot hardware.
			void read();

			/// \brief write the command to the robot hardware.
			void write(ros::Duration elapsed_time);

		protected:
			tr1cpp::TR1 tr1;
			ros::NodeHandle nh_;

			// Timing
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			double loop_hz_;

			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

			// Simulated controller
			double p_error_, v_error_, e_error_;

	};

}

#endif
