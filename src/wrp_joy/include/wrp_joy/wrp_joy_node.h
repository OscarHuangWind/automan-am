#ifndef TELEOPWR_H_
#define TELEOPWR_H_

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "human_robot_interaction/human_robot_system.h"

class TeleopWR
{
public:

    static boost::shared_ptr<TeleopWR> SINGLETON();

    ~TeleopWR(){
    	std::cout << "Remote Control Finished!" << std::endl;
    };

    ros::Publisher GetPublisher();

//    ros::Publisher GetTestPublisher();

//    boost::shared_ptr<human_robot_interaction::HRS> GetInstance();

private:
    TeleopWR();

    TeleopWR(const TeleopWR&);

    TeleopWR& operator=(const TeleopWR);

    static boost::shared_ptr<TeleopWR> instance_;
    static std::mutex mutex_;

    double pedal_;
    double steering_;

private:

    void adas_CB(const geometry_msgs::Twist::ConstPtr& twist);

    void odom_CB(const nav_msgs::Odometry::ConstPtr& odom);

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void ultraCallback(const sensor_msgs::Range::ConstPtr& ultra0, 
                     const sensor_msgs::Range::ConstPtr& ultra1, 
                     const sensor_msgs::Range::ConstPtr& ultra2, 
                     const sensor_msgs::Range::ConstPtr& ultra3);
    
    ros::NodeHandle nh_;
    
    int acc_idx_, dec_idx_,angular_idx_, flag_=0;
    double l_scale_, a_scale_;
    double safe_dist_;
    double safe_scale_ = 0.05;
    double velo_x_ = 0.0;
    double velo_limit_ = 0.5;
    double acc_limit_ = 0.5;
    double dec_default_ = 0.1;
    double threshold_ = -0.5;
    //ros subscriber and publisher
    ros::Publisher wr_pub_;
    ros::Publisher adas_trigger_pub_;
    //ros::Publisher test_pub_;
    ros::Subscriber cmd_adas_sub_;
    ros::Subscriber odom_sub_;
    
    ros::Subscriber joy_sub_;
    message_filters::Subscriber<sensor_msgs::Range> ultra0_;
    message_filters::Subscriber<sensor_msgs::Range> ultra1_;
    message_filters::Subscriber<sensor_msgs::Range> ultra2_;
    message_filters::Subscriber<sensor_msgs::Range> ultra3_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sync_pol;
    message_filters::Synchronizer<sync_pol> sync;

    boost::shared_ptr<human_robot_interaction::HRS> HRS_ptr_;
    std::list<geometry_msgs::Twist> twist_buffer_;
    std::mutex velocity_mutex_;
};

#endif