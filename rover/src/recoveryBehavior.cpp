#include<std_msgs/Bool.h>
#include<ros/ros.h>
#include<iostream>

class recovery{
    private:
        ros::Publisher pub_vel_;
        ros::Subscriber sub_initiate_recovery_;
        
    public:
    recovery(ros::NodeHandle *nh){
        sub_initiate_recovery_ = nh->subscribe("/intiate_recovery", 1, &recovery::callback, this);
    }
        void callback(const std_msgs::Bool& input);

    };

    void recovery::callback(const std_msgs::Bool& input){
        
    }
