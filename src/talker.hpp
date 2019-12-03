#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include "sensor_msgs/FluidPressure.h"

#include <iostream>
#include <fstream>
#include <sstream>

class PID
{
    public:
        PID(float desired_value, float min, float max, std::string conf_file)
            :desired_value_{desired_value}, min_{min}, max_{max}
            ,conf_file_{conf_file}
        {
            actual_value_ = 0.0;
            error_ = 0.0;
            error_prior_ = 0.0;
            integral_ = 0.0;
            derivative_ = 0.0;
            read_file(conf_file_, KP_, KI_, KD_);
        };

        void read_file(std::string filename, float& KP, float& KI, float& KD)
        {
            std::ifstream infile(filename);
            if (!infile)
            {
                ROS_INFO("Can't open the file !!! %s", filename.c_str());
                exit(1);
            }
            infile >> KP >> KI >> KD;
        }

        float get_integral()
        {
            return integral_;
        }

    private:
        float KP_, KI_, KD_;
        float desired_value_;
        float min_, max_;
        std::string conf_file_;

        float actual_value_;
        float error_prior_;
        float integral_;
        float error_;
        float derivative_;
};
