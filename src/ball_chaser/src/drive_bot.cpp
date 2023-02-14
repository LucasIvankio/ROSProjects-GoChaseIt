//Node to drive the robot around.

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

class PubAndSub
{
private:
	ros::Publisher motor_command_publisher;
	ros::NodeHandle n;
	geometry_msgs::Twist motor_command;

public:
	PubAndSub()
	{
		motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	}
	void pub_and_sub()
	{
		ROS_INFO("Publishing velocities!");
		motor_command_publisher.publish(motor_command);
	}
	
	void setMotors(float linear_x, float angular_z)
	{
		motor_command.linear.x = linear_x;
		motor_command.angular.z = angular_z;
	}
		
};

bool handle_drive_request_callback(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
	PubAndSub PASObj;
	ROS_INFO("Request to follow object with velocities linear x: %1.2f, and angular z: %1.2f", (float)req.linear_x, (float)req.angular_z);
	PASObj.setMotors(req.linear_x, req.angular_z);
	PASObj.pub_and_sub();
	
	res.msg_feedback = "Velocities set to - linear x: " + std::to_string(req.linear_x) + " , angular z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle n1;
	
	PubAndSub PASObjtInit;
	
	ros::ServiceServer service = n1.advertiseService("/ball_chaser/command_robot", handle_drive_request_callback);
	ROS_INFO("Ready to send velocities to the Robot!");
	
	// Handle ROS communication events
    ros::spin();
	
}
