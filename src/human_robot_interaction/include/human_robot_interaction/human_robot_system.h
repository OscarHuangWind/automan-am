#ifndef _HRS_H_
#define _HRS_H_

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace human_robot_interaction {
    /**
     * @class HRS
     * @brief A class that handle human robot interaction operations.
     */
    class HRS
    {
    public:
        /**
         * @brief  Constructor for the HRS
         */
        HRS();

        /**
         * @brief  Destructor
         */
        ~HRS();

        /**
         * @brief  Set cmd from hunter to parameter of this class
         * @param  cmd_vel_adas twist message from hunter
         */
        void SetParameter(const geometry_msgs::Twist::ConstPtr& cmd_vel_adas);

        /**
         * @brief  Calculate final velocity command through human robot interaction system
         * @param  vel_cmd_final final velocity command
         * @return True if no exception, false otherwise
         */
        bool CalFinalVelocityCmd(geometry_msgs::Twist& vel_cmd_final);

        /**
         * @brief  Get the twist parameter
         * @return twist message
         */
        geometry_msgs::Twist GetVelocityCmd();

    private:

        /**
         * @brief initialize the parameters
         */
        void Reset();

        /**
         * @brief calculate velocity
         */
        void CalculateVelocity(geometry_msgs::Twist& vel_cmd);

        geometry_msgs::Twist vel_adas_;

        double weight_adas_cmd_lon_, weight_adas_cmd_rot_, weight_driver_cmd_lon_, weight_driver_cmd_rot_;

    };
};

#endif