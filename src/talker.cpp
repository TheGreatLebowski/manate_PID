#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include "sensor_msgs/FluidPressure.h"

#include <sstream>

#define DESIRED_DEPTH 15.00
#define KP 0.3

float depth;
float angle = 0.0;

void chatterCallback(const sensor_msgs::FluidPressure& msg)
{
        float pressure = msg.fluid_pressure;
        //Calcul for depth
        depth = (pressure - 101.325) / 9.80638;
        //Display results
        ROS_INFO("Depth is: %f, desired depth is %f", depth, DESIRED_DEPTH);
        ROS_INFO("And angle is : [%f]", angle);
}

int error()
{
    int error = DESIRED_DEPTH - depth;
    return KP * error;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

        //PUBLISHERS
	ros::Publisher chatter_pub1 = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/submarine/fins/0/input", 1000);
	ros::Publisher chatter_pub2 = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/submarine/fins/1/input", 1000);

        //SUBSCRIBER
        ros::Subscriber sub = n.subscribe("/submarine/pressure", 1000, chatterCallback);

	ros::Rate loop_rate(10);

	float Kp = 0.3;

	while (ros::ok())
	{
            //CALCULATE ERROR
            angle = error();

            //MSG AND PUBLISH
            uuv_gazebo_ros_plugins_msgs::FloatStamped msg;

            msg.data = 0.3;

            chatter_pub1.publish(msg); 
            chatter_pub2.publish(msg); 

            ros::spinOnce();
            loop_rate.sleep();
        }
        return 0;
}
