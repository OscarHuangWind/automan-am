#include <human_robot_system.h>

namespace human_robot_interaction {

    HRS::HRS() :
    weight_adas_cmd_lon_(0.0),
    weight_adas_cmd_rot_(0.0),
    weight_driver_cmd_lon_(1.0),
    weight_driver_cmd_rot_(1.0)
    {
        vel_adas_.linear.x  = 999;
        vel_adas_.angular.z = 999;
    }

    HRS::~HRS(){}

    void HRS::SetParameter(const geometry_msgs::Twist::ConstPtr& cmd_vel_adas)
    {
        if (cmd_vel_adas == NULL)
        {
            weight_adas_cmd_lon_ = 0.0;
            weight_adas_cmd_rot_ = 0.0;
            ROS_WARN("Oscar::No command from ADAS.");
        }

        vel_adas_ = *cmd_vel_adas;
    }

    bool HRS::CalFinalVelocityCmd(geometry_msgs::Twist& vel_cmd_final)
    {
        //Reset();

        bool flag = (std::fabs(vel_adas_.linear.x)  > 100.0 && std::fabs(vel_adas_.angular.z) > 100.0);

        if (flag)
        {
            weight_adas_cmd_lon_   = 0.0;
            weight_adas_cmd_rot_   = 0.0;
            weight_driver_cmd_lon_ = 1.0 - weight_adas_cmd_lon_;
            weight_driver_cmd_rot_ = 1.0 - weight_adas_cmd_rot_;
        }
        
        if (!flag)
        {
            // weight_adas_cmd_lon_   = 1.0;
            // weight_adas_cmd_rot_   = 1.0;
            // weight_driver_cmd_lon_ = 1.0 - weight_adas_cmd_lon_;
            // weight_driver_cmd_rot_ = 1.0 - weight_adas_cmd_rot_;                       
            if (std::fabs(vel_cmd_final.angular.z - vel_adas_.angular.z) > 0.3)
            {
                //ROS_WARN("Oscar::The rotational velo is: %f, %f", vel_cmd_final.angular.z, vel_adas_.angular.z);
                weight_adas_cmd_rot_   = 1.0;
                weight_driver_cmd_rot_ = 1.0 - weight_adas_cmd_rot_;
            }
            else
            {
                //ROS_WARN("Oscar::THE rotational velo is: %f, %f", vel_cmd_final.angular.z, vel_adas_.angular.z);
                weight_adas_cmd_rot_   = 0.5;
                weight_driver_cmd_rot_ = 1.0 - weight_adas_cmd_rot_;                
            }

            if (std::fabs(vel_cmd_final.linear.x - vel_adas_.linear.x) > 0.3)
            {
                weight_adas_cmd_lon_   = 1.0;
                weight_driver_cmd_lon_ = 1.0 - weight_adas_cmd_lon_;
            }
            else
            {
                weight_adas_cmd_lon_   = 0.5;
                weight_driver_cmd_lon_ = 1.0 - weight_adas_cmd_lon_;                
            }

            if (vel_adas_.linear.x < 0)
            {
                vel_adas_.linear.x = 0.0;
            }
        }

        CalculateVelocity(vel_cmd_final);

        return true;
    }

    void HRS::Reset()
    {
        weight_adas_cmd_lon_   = 0.0;
        weight_adas_cmd_rot_   = 0.0;
        weight_driver_cmd_lon_ = 1.0 - weight_adas_cmd_lon_;
        weight_driver_cmd_rot_ = 1.0 - weight_adas_cmd_rot_;
    }

    void HRS::CalculateVelocity(geometry_msgs::Twist& vel_cmd)
    {
        ROS_WARN("Oscar::the weight of adas is: %f, %f", weight_adas_cmd_lon_, weight_adas_cmd_rot_);
        vel_cmd.linear.x  = weight_adas_cmd_lon_ * vel_adas_.linear.x + weight_driver_cmd_lon_ * vel_cmd.linear.x;
        vel_cmd.angular.z = weight_adas_cmd_rot_ * vel_adas_.angular.z + weight_driver_cmd_rot_ * vel_cmd.angular.z;
    }

    geometry_msgs::Twist HRS::GetVelocityCmd()
    {
        return vel_adas_;
    }

};