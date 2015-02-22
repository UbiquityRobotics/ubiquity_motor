#include <ros/ros.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ubiquity_motor/MotorCallbackInterface.h>
#include <ubiquity_motor/MotorCommand.h>

class MotorDriver : public hardware_interface::RobotHW, public MotorCallbackInterface
{
	public:
		MotorDriver();
		~MotorDriver();

		void write();

		void mcbiCallbackFunction(MotorCommand command);

	private:
		hardware_interface::JointStateInterface    jnt_state_interface_;
		hardware_interface::VelocityJointInterface jnt_vel_interface_;
		double cmd_[2]; 
		double pos_[2];
		double vel_[2];
		double eff_[2];
}