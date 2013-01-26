#include "joy_wrapper.h"

Teleop_joy_wrapper::Teleop_joy_wrapper()//: linear_vel(1), angular_vel(2)
{
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop_joy_wrapper::joyCallback, this);
  axes_vel_pub = nh.advertise<geometry_msgs::Twist>("cc_node/CartesianParameters", 1);
  gripperPositionPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
  gripperValue = 0.0;
}

void Teleop_joy_wrapper::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	ros::Rate loop_rate(20);
	
    //######## gripper
    std::vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(2);
    std::cout << "gripper Value = " << gripperValue << "    ,    "<< "BUTTON 9= "<< joy->buttons[BUTTON9]<< "   BUTTON 10= "<< joy->buttons[BUTTON10]<<std::endl;
	if(joy->buttons[BUTTON9] && gripperValue != 0.0)
	{
		gripperValue = 0.0;
		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
		gripperJointPositions[0].value = gripperValue;
		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
		gripperJointPositions[1].value = gripperValue;
		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);
		std::cout << "gripperValue = " << gripperValue << std::endl;
 		command.positions = gripperJointPositions;
        gripperPositionPublisher.publish(command);
        loop_rate.sleep();
	}else if(joy->buttons[BUTTON10] && gripperValue != 0.0115)
	{
		gripperValue = 0.0115;
		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
		gripperJointPositions[0].value = gripperValue;
		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
		gripperJointPositions[1].value = gripperValue;
		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);
		std::cout << "gripperValue = " << gripperValue << std::endl;
		command.positions = gripperJointPositions;
        gripperPositionPublisher.publish(command);
        loop_rate.sleep();
	}
	
	//######## end of gripper 
		
  geometry_msgs::Twist twist;	
  
  twist.linear.x = joy->buttons[BUTTON7]* joy->axes[0];
  twist.linear.y = joy->buttons[BUTTON7]* joy->axes[1];
  twist.linear.z = joy->buttons[BUTTON7]* joy->axes[3];
   
  twist.angular.x = joy->buttons[BUTTON7]* joy->axes[5];
  twist.angular.y = joy->buttons[BUTTON7]* joy->axes[4];
  twist.angular.z = joy->buttons[BUTTON7]* joy->axes[2];
    //std::vector<double> angular = {x_ang_vel, y_ang_vel, z_ang_vel};
  
  
  axes_vel_pub.publish(twist);
  
}

//*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_wrapper");
	Teleop_joy_wrapper joy_wrapper;
	while(ros::ok()) {	
		ros::spin();
	}
}
//*/
