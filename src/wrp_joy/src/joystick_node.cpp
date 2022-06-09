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

class JoyStick
{
public:
    JoyStick();
    ~JoyStick(){
    	std::cout << "Remote Control Finished!" << std::endl;
    };
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void ultrasonicCallback(const sensor_msgs::Range::ConstPtr& ultra0, 
							const sensor_msgs::Range::ConstPtr& ultra1, 
							const sensor_msgs::Range::ConstPtr& ultra2, 
							const sensor_msgs::Range::ConstPtr& ultra3,
							const sensor_msgs::Range::ConstPtr& ultra4, 
							const sensor_msgs::Range::ConstPtr& ultra5, 
							const sensor_msgs::Range::ConstPtr& ultra6, 
							const sensor_msgs::Range::ConstPtr& ultra7  );

    ros::NodeHandle nh_;
    
    int linear_idx_, angular_idx_, flag=0;
    double l_scale_, a_scale_;
    double safe_dist_ ;
    double psafe_scale_ = 1;
    double nsafe_scale_ = 1;
    //ros subscriber and publisher
    ros::Publisher wr_pub_;
    //ros::Subscriber joy_sub_;
    message_filters::Subscriber<sensor_msgs::Joy> joy_sub_;
    message_filters::Subscriber<sensor_msgs::Range> ultra0_;
    message_filters::Subscriber<sensor_msgs::Range> ultra1_;
    message_filters::Subscriber<sensor_msgs::Range> ultra2_;
    message_filters::Subscriber<sensor_msgs::Range> ultra3_;
	message_filters::Subscriber<sensor_msgs::Range> ultra4_;
    message_filters::Subscriber<sensor_msgs::Range> ultra5_;
    message_filters::Subscriber<sensor_msgs::Range> ultra6_;
    message_filters::Subscriber<sensor_msgs::Range> ultra7_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sync_pol;
    message_filters::Synchronizer<sync_pol> sync;

};

JoyStick::JoyStick():linear_idx_(1),
angular_idx_(2),
l_scale_(0.5),
a_scale_(0.3),
safe_dist_(0.2),
joy_sub_(nh_, "/joy", 100),
ultra0_(nh_, "/ultrasonic0", 100),
ultra1_(nh_, "/ultrasonic1", 100),
ultra2_(nh_, "/ultrasonic2", 100),
ultra3_(nh_, "/ultrasonic3", 100),
ultra4_(nh_, "/ultrasonic4", 100),
ultra5_(nh_, "/ultrasonic5", 100),
ultra6_(nh_, "/ultrasonic6", 100),
ultra7_(nh_, "/ultrasonic7", 100),
sync(sync_pol(10), ultra0_, ultra1_, ultra2_, ultra3_, ultra4_, ultra5_, ultra6_, ultra7_)
{
	std::cout << "Remote Contorl Node" << std ::endl;
 
    wr_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	
	joy_sub_.registerCallback(boost::bind(&JoyStick::joyCallback,this,_1));
	sync.registerCallback(boost::bind(&JoyStick::ultrasonicCallback,this,_1,_2,_3,_4,_5,_6,_7,_8));
	
}

void JoyStick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{	
	if(joy->buttons[0]==1)
		flag = 1;
		std::cout << "Hunter will start!" << std::endl;
	//start event lock
    geometry_msgs::Twist twist;
	if(flag){
		if (joy->axes[linear_idx_] >= 0){
		twist.angular.z = a_scale_ * joy->axes[angular_idx_];
		twist.linear.x = psafe_scale_ * l_scale_ * joy->axes[linear_idx_];
		}
		else
		{
			twist.angular.z = a_scale_ * joy->axes[angular_idx_];
			twist.linear.x = nsafe_scale_ * l_scale_ * joy->axes[linear_idx_];
		}
	}
    wr_pub_.publish(twist);
    
}

void JoyStick::ultrasonicCallback(const sensor_msgs::Range::ConstPtr& ultra0, 
const sensor_msgs::Range::ConstPtr& ultra1, 
const sensor_msgs::Range::ConstPtr& ultra2, 
const sensor_msgs::Range::ConstPtr& ultra3,
const sensor_msgs::Range::ConstPtr& ultra4, 
const sensor_msgs::Range::ConstPtr& ultra5, 
const sensor_msgs::Range::ConstPtr& ultra6, 
const sensor_msgs::Range::ConstPtr& ultra7  ){
	psafe_scale_ = 1;
	nsafe_scale_ = 1;	
	
	if(((ultra0->range < safe_dist_ ) || (ultra1->range < safe_dist_) || (ultra2->range < safe_dist_) || (ultra3->range < safe_dist_)) && ((ultra4->range > safe_dist_) || (ultra5->range > safe_dist_) || (ultra6->range > safe_dist_) || (ultra7->range > safe_dist_)) ){
		psafe_scale_ = 0;
		nsafe_scale_ = 1;
	}
	else if( ((ultra4->range < safe_dist_ ) || (ultra5->range < safe_dist_) || (ultra6->range < safe_dist_) || (ultra7->range < safe_dist_)) && ((ultra0->range > safe_dist_) || (ultra1->range > safe_dist_) || (ultra2->range > safe_dist_) || (ultra3->range > safe_dist_)) )
	{
		psafe_scale_ = 1;
		nsafe_scale_ = 0;	
	}
	else if (((ultra0->range < safe_dist_ ) || (ultra1->range < safe_dist_) || (ultra2->range < safe_dist_) || (ultra3->range < safe_dist_)) && ((ultra4->range < safe_dist_) || (ultra5->range < safe_dist_) || (ultra6->range < safe_dist_) || (ultra7->range < safe_dist_)) )
	{
		psafe_scale_ = 0;
		nsafe_scale_ = 0;	
	}
}

int main(int argc, char** argv){
    ros::init(argc,argv, "joystick");
    JoyStick joystick;
    ros::Rate rate(30);
    
    while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
}
