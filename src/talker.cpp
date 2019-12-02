#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include "sensor_msgs/FluidPressure.h"

#include <iostream>
#include <fstream>
#include <sstream>

#define DESIRED_DEPTH 15.00
#define KP -0.3
#define KI 0.5
#define KD 0.4
#define MAXIMUM_ANGLE 0.6
#define MINIMUM_ANGLE -0.6
#define epsi 0.01
#define dt 0.01

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

float output(float result_error)
{
    if (abs(result_error) > MAXIMUM_ANGLE)
        return result_error > 0 ? MAXIMUM_ANGLE : MINIMUM_ANGLE * -1;
    return result_error;
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


    static float error_prior = 0.0;
    static float integral = 0.0;
    float error = 0.0;
    float derivative = 0.0;
    
    while (ros::ok())
    {
        //CALCULATE ERROR
        error = DESIRED_DEPTH - depth;

        if (abs(error) > epsi)
            integral += error * dt;

        derivative = (error - error_prior) / dt;

        angle = KP * error + KI * integral + KI * derivative;

        error_prior = error;

        //MSG AND PUBLISH
        uuv_gazebo_ros_plugins_msgs::FloatStamped msg; //Creation of msg

        msg.data = output(angle); //If the angle is to big, reduce it

        chatter_pub1.publish(msg); 
        chatter_pub2.publish(msg); 

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
