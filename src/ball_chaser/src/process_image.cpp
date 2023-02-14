//Node to process the image and determine the ball position
//Then, request the robot to move

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>



class PubAndSub
{
private:
	ros::ServiceClient client;
	ros::Subscriber sub1;
	ros::NodeHandle n;
	ball_chaser::DriveToTarget srv;
	float lin_x, ang_z;

public:
	PubAndSub()
	{
		client = n.serviceClient<ball_chaser::DriveToTarget>("ball_chaser/command_robot");
		sub1 = n.subscribe("/camera/rgb/image_raw", 10, &PubAndSub::process_image_callback, this);
	}
	void drive_robot(float linear_x, float angular_z)
	{
		ROS_INFO_STREAM("Driving the robot towards the ball");
	
		srv.request.linear_x = linear_x;
		srv.request.angular_z = angular_z;
	
		if (!client.call(srv))
			ROS_ERROR("Failed to call service drive_bot");
	}
	
	void process_image_callback(const sensor_msgs::Image img)
	{
		int white_pixel=255;
		int i = 0;
		bool is_ball = false;
		
		for(i = 0; i < img.height * img.step; i++)
		{
			if (img.data[i] == white_pixel)
			{
				is_ball = true;
				break;
			}
			else is_ball = false;
		}
		if (is_ball && i%img.step < (int)img.step/3)
		{
			lin_x=0;
			ang_z=0.5;
		}
		else if (is_ball && i%img.step > (int)img.step*2/3))
		{
			lin_x=0;
			ang_z=-0.5;
		}
		else if(is_ball)
		{
			lin_x=0.5;
			ang_z=0;
		}
		else
		{
			lin_x=0;
			ang_z=0;
		}
		drive_robot(lin_x, ang_z);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "process_image");
	
	PubAndSub PASObj;
	
	ros::spin();
}


