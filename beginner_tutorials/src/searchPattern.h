//Aditya Arun Iyer Contact - 8073711953   Mail - adityaaruniyer01@gmail.com
//Vignesh S. Iyer   Contact - 7506017898   Mail - vignesh02iyer@gmail.com


#include "iostream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "math.h"
#include "time.h"
#include <string>


namespace sp
{
    class searchPattern
    {
        private:
        
	    ros::Subscriber sub_imu;
	    
	    ros::Subscriber sub_id;
	    
	    ros::Publisher pub_vel;
	    
	    double yaw_set,yaw;
	    
	    ros::Time curr_time;
	    
	    int count;
	    
	    bool detected =0;
	    
	    bool reset_angle = 1;
	    
	    bool forward =0;
	    
	    bool shutupanddontturn = 0;
	    
	    geometry_msgs::Twist Vel;
	    
	    double elapsed_time;
            //Node Handle
            ros::NodeHandle nh; 


        public:
            //Class Constructor
            searchPattern(); 
            //IMU Callback function
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

            void distance();

            void rotation();

            double angle_correction(double angle);

            void set_angle();
            
            void move_forward();
    };
}



