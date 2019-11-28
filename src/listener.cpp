#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/FluidPressure.h"

float depth;

void chatterCallback(const sensor_msgs::FluidPressure& msg)
{
	float pressure = msg.fluid_pressure;
	//Calcul for depth
	depth = (pressure - 101.325) / 9.80638;
	//Display results
	ROS_INFO("Depth is: [%f]", depth);
	ROS_INFO("And pressure is : [%f]", pressure);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/submarine/pressure", 1000, chatterCallback);

	ros::spin();

	return 0;
}
