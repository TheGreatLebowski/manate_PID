#include "talker.hpp"

PID fins = PID(15, -0.6, 0.6, "pid.conf", "depth.csv", "TIME,DEPTH", 
        "/submarine/pressure", "/submarine/fins/0/input",
        "/submarine/fins/1/input");

void chatterCallback(const sensor_msgs::FluidPressure& msg)
{
    float pressure = msg.fluid_pressure;
    //Calcul for depth
    fins.set_actual_value((pressure - 101.325) / 9.80638);
    //Display results
    ROS_INFO("Depth is: %f", fins.get_actual_value());
    ROS_INFO("And angle is : [%f]", fins.get_output());
}

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    //SUBSCRIBER
    ros::Subscriber sub = n.subscribe(fins.get_subscribe(), 1000, chatterCallback);

    ros::Rate loop_rate(10);
    float time = 0.0;
    while (ros::ok())
    {
        fins.publish_file(time);
    
        time += 0.01;

        //CALCULATE ERROR
        fins.PID_calcul();

        //MSG AND PUBLISH
        uuv_gazebo_ros_plugins_msgs::FloatStamped msg; //Creation of msg

        msg.data = fins.limit(); //If the angle is to big, reduce it

        fins.publish(n, msg);

        ros::spinOnce();
       
        //loop_rate.sleep();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
