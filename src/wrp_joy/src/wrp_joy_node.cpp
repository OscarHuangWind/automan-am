#include <iostream>
#include <mutex>
#include <boost/shared_ptr.hpp>

#include <wrp_joy_node.h>

const float MIN_RANGE = 0.2000;
const float MAX_RANGE = 0.9000;


boost::shared_ptr<TeleopWR> TeleopWR::SINGLETON()
{
    if (!instance_)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ == nullptr)
        {
            instance_ = boost::shared_ptr<TeleopWR>(new TeleopWR);
        }
    }
    return instance_;
}


TeleopWR::TeleopWR():acc_idx_(2),
angular_idx_(0),
l_scale_(0.5),
a_scale_(0.3),
safe_dist_(MAX_RANGE),
ultra0_(nh_, "/ultrasonic1", 100),
ultra1_(nh_, "/ultrasonic3", 100),
ultra2_(nh_, "/ultrasonic2", 100),
ultra3_(nh_, "/ultrasonic6", 100),
sync(sync_pol(10),ultra0_, ultra1_, ultra2_, ultra3_)
{
	std::cout << "Remote Contorl Node" << std ::endl;
    nh_.param("threshold", threshold_, threshold_);
    nh_.param("vel_limit", velo_limit_, velo_limit_);
    nh_.param("dec_default", dec_default_,dec_default_);
    nh_.param("acc_limit", acc_limit_, acc_limit_);
    nh_.param("vel_acc", acc_idx_, acc_idx_);
    nh_.param("vel_dec", dec_idx_, dec_idx_);
    nh_.param("vel_angular", angular_idx_, angular_idx_);
    nh_.param("scale_linear", l_scale_,l_scale_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("safe_distance", safe_dist_, safe_dist_);

    HRS_ptr_ = boost::shared_ptr<human_robot_interaction::HRS>(new human_robot_interaction::HRS);

    // subscriber
    cmd_adas_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel_adas", 1, boost::bind(&TeleopWR::adas_CB, this, _1));
    joy_sub_= nh_.subscribe<sensor_msgs::Joy>("/joy", 1, boost::bind(&TeleopWR::joyCallback, this, _1));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&TeleopWR::odom_CB, this, _1));

    // publiser
    wr_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    adas_trigger_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_driver", 1);
//    test_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_test", 1);

    sync.registerCallback(boost::bind(&TeleopWR::ultraCallback,this,_1,_2,_3,_4));
}

void TeleopWR::adas_CB(const geometry_msgs::Twist::ConstPtr& twist)
{
    //ROS_WARN("Oscar::Received message from hunter.%f, %f", twist->linear.x, twist->angular.z);
    HRS_ptr_->SetParameter(twist);
}

void TeleopWR::odom_CB(const nav_msgs::Odometry::ConstPtr& odom)
{
    //ROS_WARN("Oscar::Received message from hunter.%f, %f", odom->twist.twist.linear.x, odom->twist.twist.angular.z);
    {
        const std::lock_guard<std::mutex> lock(velocity_mutex_);
        velo_x_ = odom->twist.twist.linear.x;
    }
}

void TeleopWR::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[5]==1 && joy->axes[acc_idx_] == -1 && joy->axes[dec_idx_] == -1)
		flag_ = 1;
	//start event lock
    geometry_msgs::Twist twist;

    if (flag_)
    {
        twist.angular.z = a_scale_ * joy->axes[angular_idx_];
        if (joy->buttons[4] != 1)
        {
            if (joy->axes[dec_idx_] > -1)
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("The brake velocity is:%f", velo_x_);
                twist.linear.x = std::max(velo_x_ - std::min(safe_scale_ * l_scale_ * (joy->axes[dec_idx_] + 1), acc_limit_), 0.0); 
            }
            else if (joy->axes[acc_idx_] > threshold_)
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("The acc velocity is:%f", velo_x_);
                twist.linear.x = std::min(velo_x_ + std::min(safe_scale_ * l_scale_ * (joy->axes[acc_idx_] + 1), acc_limit_), velo_limit_);
            }
            else if (joy->axes[acc_idx_] > -1 && joy->axes[acc_idx_] <= threshold_)
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("The maintain velocity is:%f", velo_x_);
                twist.linear.x = std::min(velo_x_, velo_limit_);
            }
            else
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("The non-op velocity is:%f", velo_x_);
                twist.linear.x = std::max(velo_x_ - std::min(dec_default_, acc_limit_), 0.0); 
            }
        }
        else
        {
            if (joy->axes[dec_idx_] > -1)
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("THHHHE brake velocity is:%f", velo_x_);
                twist.linear.x = std::min(velo_x_ + std::min(safe_scale_ * l_scale_ * (joy->axes[dec_idx_] + 1), acc_limit_), 0.0); 
            }
            else if (joy->axes[acc_idx_] > -1)
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("THHHHE acc velocity is:%f", velo_x_);
                twist.linear.x = std::max(velo_x_ - std::min(safe_scale_ * l_scale_ * (joy->axes[acc_idx_] + 1), acc_limit_), -velo_limit_);
            }
            else
            {
                const std::lock_guard<std::mutex> lock(velocity_mutex_);
                //ROS_WARN("THHHHE non-op velocity is:%f", velo_x_);
                twist.linear.x = std::min(velo_x_ + std::min(dec_default_, acc_limit_), 0.0); 
            }
        }
        twist_buffer_.push_back(twist);
    }

    // if (flag_){
	// 	twist.angular.z = a_scale_ * joy->axes[angular_idx_];
		
	// 	if (joy->buttons[4]==0)//
	// 	    twist.linear.x = safe_scale_ * l_scale_ * (joy->axes[acc_idx_] + 1);
	// 	else
	// 	    twist.linear.x = -1 * safe_scale_ * l_scale_ * (joy->axes[acc_idx_] + 1);

    //     twist_buffer_.push_back(twist);
    // }

    //ROS_WARN("Oscar::the driver velocity is: %f, %f", twist.linear.x, twist.angular.z);

    double cmd_vel_lon_sum = 0.0;
    double cmd_vel_rot_sum = 0.0;

    if (twist_buffer_.size() > 10)
    {
        twist_buffer_.pop_front();
    }

    for (auto it = twist_buffer_.begin(); it != twist_buffer_.end(); ++it)
    {
        cmd_vel_lon_sum += it->linear.x;
        cmd_vel_rot_sum += it->angular.z;
    }

    twist.linear.x  = cmd_vel_lon_sum / std::max(int(twist_buffer_.size()), 1);
    twist.angular.z = cmd_vel_rot_sum / std::max(int(twist_buffer_.size()), 1); 

    //ROS_WARN("Oscar::the flattened driver velocity is: %f, %f", twist.linear.x, twist.angular.z);

    adas_trigger_pub_.publish(twist);

    HRS_ptr_->CalFinalVelocityCmd(twist);

    //ROS_WARN("Oscar::THE final velocity is: %f, %f", twist.linear.x, twist.angular.z);

    wr_pub_.publish(twist);
    
}

void TeleopWR::ultraCallback(const sensor_msgs::Range::ConstPtr& ultra0, 
                           const sensor_msgs::Range::ConstPtr& ultra1, 
                           const sensor_msgs::Range::ConstPtr& ultra2, 
                           const sensor_msgs::Range::ConstPtr& ultra3 
)
{
	if ((ultra0->range > MIN_RANGE && ultra0->range < safe_dist_) ||
    (ultra1->range > MIN_RANGE && ultra1->range < safe_dist_) ||
    (ultra2->range > MIN_RANGE && ultra2->range < safe_dist_) ||
    (ultra3->range > MIN_RANGE && ultra3->range < safe_dist_))
    	safe_scale_ = 0.05;
    else
    	safe_scale_ = 1;
}


ros::Publisher TeleopWR::GetPublisher()
{
    return adas_trigger_pub_;
}

boost::shared_ptr<TeleopWR> TeleopWR::instance_ = NULL;
std::mutex TeleopWR::mutex_;

int main(int argc, char** argv){
    ros::init(argc,argv, "teleop_wr");
    boost::shared_ptr<TeleopWR> teleop_wr = TeleopWR::SINGLETON();
    ros::Rate rate(100);
    while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
}
