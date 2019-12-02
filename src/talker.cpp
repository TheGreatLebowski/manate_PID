#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include "sensor_msgs/FluidPressure.h"

#include <iostream>
#include <fstream>
#include <sstream>

#define DESIRED_DEPTH 15.00
#define MAXIMUM_ANGLE 0.6
#define MINIMUM_ANGLE -0.6
#define epsi 0.01
#define dt 0.01

float depth;
float angle = 0.0;
float KP, KI, KD;


void read_file(std::string filename)
{
    std::ifstream infile(filename);
    if (!infile)
    {
        ROS_INFO("Can't open the file %s", filename.c_str());
        exit(1);
    }
    infile >> KP >> KI >> KD;
    ROS_INFO("KP = %f, KI = %f, KD = %f", KP, KI, KD);
    infile.close();

}

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
        return result_error > 0 ? MAXIMUM_ANGLE : MINIMUM_ANGLE;
    return result_error;
}

int main(int argc, char **argv)
{
    read_file("pid.conf");
    
    std::ofstream file;
    file.open("depth.csv", std::ios_base::app);
    file << "TIME,DEPTH"<< std::endl;

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
    

    float time = 0.0;
    while (ros::ok())
    {
        file << time << "," << depth << std::endl;
        time += 0.01;
        //CALCULATE ERROR
        error = DESIRED_DEPTH - depth;

        if (abs(error) > epsi)
            integral += error * dt;

        derivative = (error - error_prior) / dt;

        angle = KP * error + KI * integral + KD * derivative;

        error_prior = error;

        //MSG AND PUBLISH
        uuv_gazebo_ros_plugins_msgs::FloatStamped msg; //Creation of msg

        msg.data = output(angle); //If the angle is to big, reduce it

        chatter_pub1.publish(msg); 
        chatter_pub2.publish(msg);

        ros::spinOnce();
        //loop_rate.sleep();
        ros::Duration(0.01).sleep();
    }
    file.close();
    return 0;
}
