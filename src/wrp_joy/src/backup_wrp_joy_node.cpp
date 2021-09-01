#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>

const float MIN_RANGE = 0.2000;
const float MAX_RANGE = 0.9000;

class TeleopWR
{
public:
    TeleopWR();
    ~TeleopWR(){
    	std::cout << "Remote Control Finished!" << std::endl;
    };
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, 
const sensor_msgs::Range::ConstPtr& ultra0, 
const sensor_msgs::Range::ConstPtr& ultra1, 
const sensor_msgs::Range::ConstPtr& ultra2, 
const sensor_msgs::Range::ConstPtr& ultra3);
    
    ros::NodeHandle nh_;
    
    int linear_idx_, angular_idx_, flag=0;
    double l_scale_, a_scale_;
    double safe_dist_;
    double safe_scale_ = 0.05;
    //ros subscriber and publisher
    ros::Publisher wr_pub_;
    //ros::Subscriber joy_sub_;
    
    message_filters::Subscriber<sensor_msgs::Joy> joy_sub_;
    message_filters::Subscriber<sensor_msgs::Range> ultra0_;
    message_filters::Subscriber<sensor_msgs::Range> ultra1_;
    message_filters::Subscriber<sensor_msgs::Range> ultra2_;
    message_filters::Subscriber<sensor_msgs::Range> ultra3_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Joy, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sync_pol;
    message_filters::Synchronizer<sync_pol> sync;
};

TeleopWR::TeleopWR():linear_idx_(2),
angular_idx_(0),
l_scale_(0.5),
a_scale_(0.3),
safe_dist_(MAX_RANGE),
joy_sub_(nh_, "joy", 100),
ultra0_(nh_, "/ultrasonic1", 100),
ultra1_(nh_, "/ultrasonic3", 100),
ultra2_(nh_, "/ultrasonic2", 100),
ultra3_(nh_, "/ultrasonic6", 100),
sync(sync_pol(10),joy_sub_, ultra0_, ultra1_, ultra2_, ultra3_)
{
	std::cout << "Remote Contorl Node" << std ::endl;
    nh_.param("vel_linear", linear_idx_, linear_idx_);
    nh_.param("vel_angular", angular_idx_, angular_idx_);
    nh_.param("scale_linear", l_scale_,l_scale_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("safe_distance", safe_dist_, safe_dist_);
    
    wr_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    
    sync.registerCallback(boost::bind(&TeleopWR::joyCallback,this,_1,_2,_3,_4,_5));
}

void TeleopWR::joyCallback(const sensor_msgs::Joy::ConstPtr& joy, 
const sensor_msgs::Range::ConstPtr& ultra0, 
const sensor_msgs::Range::ConstPtr& ultra1, 
const sensor_msgs::Range::ConstPtr& ultra2, 
const sensor_msgs::Range::ConstPtr& ultra3 
)
{
	
	if(joy->buttons[5]==1 && joy->axes[linear_idx_] == -1)
		flag = 1;
	//start event lock
    geometry_msgs::Twist twist;
   
    if ((ultra0->range > MIN_RANGE && ultra0->range < safe_dist_) ||
    (ultra1->range > MIN_RANGE && ultra1->range < safe_dist_) ||
    (ultra2->range > MIN_RANGE && ultra2->range < safe_dist_) ||
    (ultra3->range > MIN_RANGE && ultra3->range < safe_dist_))
    	safe_scale_ = 0.05;
    else
    	safe_scale_ = 1;
    	
    if (flag){
		twist.angular.z = a_scale_ * joy->axes[angular_idx_];
		
		if (joy->buttons[4]==0)//
		    twist.linear.x = safe_scale_ * l_scale_ * (joy->axes[linear_idx_] + 1);
		else
		    twist.linear.x = -1 * safe_scale_ * l_scale_ * (joy->axes[linear_idx_] + 1);
    }
    wr_pub_.publish(twist);
    
}

int main(int argc, char** argv){
    ros::init(argc,argv, "teleop_wr");
    TeleopWR teleop_wr;
    ros::Rate rate(30);
    while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
}
