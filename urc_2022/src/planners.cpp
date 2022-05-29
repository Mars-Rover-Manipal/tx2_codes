#include "urc_2022/planners.h"

using namespace traversal;

planners::planners(ros::NodeHandle nh,ros::Publisher pub_vel_)
{
    this->nh = nh;
    this->pub_vel_ = pub_vel_;
    yaw_ = 0;
    goal_lat_ = 0;
    goal_long_ = 0;
    error_distance_ = 0;
    in_error_distance_ = 0;
    diff_angle_ = 0;
    error_angle_ = 0;
    linear_max_ = 0.75;
    linear_min_ =0.2;
    ang_max_ = 0.5;
    ang_min_ = 0.1;
    dist_thres_ = 1.5;
    ang_thres_ = 2;
}

void planners::caller(std::string goal_no)
{
    //std::string goal_no;
    std::vector<double> goals;
    //std::cout<<"Enter goal_no: \n";
    //std::cin>>goal_no;
    std::string goal = "/goal" + goal_no;
    nh.getParam(goal, goals);

    goal_lat_ = goals[0];
    goal_long_ = goals[1];
    
    ros::Rate loop_rate(1);
    
    pub_vel_ = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);       
            
    sub_gps_ = nh.subscribe("/fix", 1000, &planners::gpsCallback, this);         
    loop_rate.sleep();
    
    sub_imu_ = nh.subscribe("/imu", 1000, &planners::imuCallback, this);         
        
    //ros::spin();
}

void planners::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    sensor_lat_ = (msg->latitude);
    sensor_long_ = (msg->longitude);
	std::cout<<"sensor_lat_: "<<std::setprecision(16)<<sensor_lat_<<" sensor_long_: "<<std::setprecision(16)<<sensor_long_<<std::endl;
    in_error_distance_ = error_distance_;
    std::cout << error_distance_<<std::endl;
    distance();
}

void planners::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    this->yaw_ = msg->orientation.z;
    if(error_distance_ > dist_thres_)
    {
        rotation();
    }   
    else
    {
        char input;
        vel_.linear.x = 0;
        vel_.angular.z = 0;
        pub_vel_.publish(vel_);
        std::cout<<"Goal Reached"<<std::endl;
        std::cout<<"Do you want to go to another goal (y or n)?"<<std::endl;
        std::cin>>input;
        if(input == 'y')
            nh.setParam("/goalReached", 1);
        else
            ros::shutdown();       
    }
}


void planners::distance()
{
    double diff_long = goal_long_ - sensor_long_;
	double diff_lat = goal_lat_ - sensor_lat_;
	double sensor_lat_deg = sensor_lat_ * M_PI / 180.0;
    double y = sin(diff_long) * cos(goal_lat_);
    double x = cos(sensor_lat_) * sin(goal_lat_) - sin(sensor_lat_) * cos(goal_lat_) * cos(diff_long);
    error_angle_ = atan2(y, x);
    error_angle_ = -error_angle_ * SEMI_CIRCLE / M_PI;

    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>> spherical_point;
    spherical_point p(sensor_long_, sensor_lat_);
    spherical_point q(goal_long_, goal_lat_);
    double dist = boost::geometry::distance(p, q);
    error_distance_ = dist*EARTH_RADIUS*1000;
    std::cout<<"distance is "<<error_distance_<<std::endl;
}

void planners::rotation()
{   
    float ang_z, vel_x;
    pid_ l,a;
    l.kp = 0.05, l.kd = 0.06;
    a.kp = 0.75, a.kd = 0;                

    if(yaw_<0)
        yaw_ = yaw_ + CIRCLE;
    if(error_angle_<0)
        error_angle_ = error_angle_ + CIRCLE;
    diff_angle_ = error_angle_ - yaw_;

    double diff_error_angle_rad; 

    if(diff_angle_ < 0)
    {
        if(abs(diff_angle_)>SEMI_CIRCLE)
        {
            diff_angle_ =  diff_angle_ + CIRCLE;
            diff_error_angle_rad = abs(diff_angle_) * M_PI /SEMI_CIRCLE;               
            ang_z = diff_error_angle_rad * a.kp; 
        }
        else
        {
            diff_error_angle_rad = abs(diff_angle_) * M_PI /SEMI_CIRCLE;               
            ang_z = - diff_error_angle_rad * a.kp;
        }

    }
    else
    {
        if(abs(diff_angle_)>SEMI_CIRCLE)
        {
            diff_angle_ =  CIRCLE - diff_angle_;
            diff_error_angle_rad = abs(diff_angle_) * M_PI /SEMI_CIRCLE;               
            ang_z = - diff_error_angle_rad * a.kp;
        }
        else
        {
            diff_error_angle_rad = abs(diff_angle_) * M_PI /SEMI_CIRCLE;               
            ang_z = diff_error_angle_rad * a.kp;                                           
        }
    }

    vel_x = error_distance_ * l.kp + l.kd*(abs(in_error_distance_ - error_distance_)) / 0.1;

    if((ang_z > ang_max_) && (ang_z > 0))
        ang_z = ang_max_;

    if((ang_z < - ang_max_) && (ang_z < 0))
        ang_z = - ang_max_;



    if(vel_x > linear_max_)
        vel_x = linear_max_;
    if(vel_x < linear_min_)
        vel_x = linear_min_;

    if(abs(diff_angle_) > ang_thres_)
    {
        this->vel_.angular.z = ang_z;
        this->vel_.linear.x = vel_x;
        this->pub_vel_.publish(vel_);
    }
    else
    {
        this->vel_.linear.x = 0;
        this->vel_.angular.z = 0;
        pub_vel_.publish(vel_);
        linearMovement();
    }

}

void planners::linearMovement()
{
	float vel_x; 
    pid_ lm;
    lm.kp = 0.05,lm.kd = 0.06;  
                                              
    vel_x = error_distance_ * lm.kp + lm.kd*(abs(in_error_distance_ - error_distance_))/0.1;

    if(vel_x > linear_max_)
        vel_x = linear_max_;
    if(vel_x < linear_min_)
        vel_x = linear_min_;

    this->vel_.linear.x = vel_x;

    pub_vel_.publish(vel_);
}


