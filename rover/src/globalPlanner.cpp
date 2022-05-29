#include "globalPlanner.h"
#include "std_msgs/Float64.h"

using namespace gp;

globalPlanner::globalPlanner()
{
    yaw_ = 0;
    goal_lat_ = 0;
    goal_long_ = 0;
    distance_ = 0;
    in_distance_ = 0;
    diff_angle = 0;
    angle_ = 0;
    linear_max = 1;
    linear_min =0.2;
    ang_max = 0.75;
    ang_min = 0.1;
    dist_thres = 0.1;
    ang_thres = 1;
}

void globalPlanner::caller()
{
    std::string goal_no;
    std::vector<double> goals;
    std::cout<<"Enter goal_no: \n";
    std::cin>>goal_no;
    std::string goal = "/goal" + goal_no;
    nh.getParam(goal, goals);

    goal_lat_ = goals[0];
    goal_long_ = goals[1];

    ros::Rate loop_rate(10);
    pub_vel_ = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);  
    pub_az_  = nh.advertise<std_msgs::Float64>("/az", 1);                // Velocity Publisher
    sub_gps_ = nh.subscribe("/fix", 1000, &globalPlanner::gpsCallback, this);         //Callback for gps values
    loop_rate.sleep();
    sub_imu_ = nh.subscribe("/imu", 1000, &globalPlanner::imuCallback, this);         //Callback for imu values
    ros::spin();
}

void globalPlanner::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    sensor_lat_ = msg->latitude;
    sensor_long_ = msg->longitude;
    in_distance_ = distance_;
    distance();
}

void globalPlanner::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double roll, pitch, yaw;
    char input;
    tf::Quaternion q(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    this->yaw_ = - yaw*semi_circle/M_PI;

    if(distance_ > dist_thres)
    std::cout<<"lol";
        // rotation();
    else
    {
        vel_.linear.x = 0;
        vel_.angular.z = 0;
        pub_vel_.publish(vel_);
        std::cout<<"Goal Reached"<<std::endl;

        std::cout<<"Do you want to go to another goal (y or n)?"<<std::endl;
        std::cin>>input;
        if(input == 'y')
            caller();
        else
            ros::shutdown();       
    }
}

void globalPlanner::distance()
{
    double const earth_radius = 6371.0; 
    double diff_long = goal_long_ - sensor_long_;

    //Formula for angle calculation in degrees
    double y = sin(diff_long) * cos(goal_lat_);
    double x = cos(sensor_lat_) * sin(goal_lat_) - sin(sensor_lat_) * cos(goal_lat_) * cos(diff_long);
    angle_ = atan2(y, x);
    angle_ = angle_ * semi_circle / M_PI;
    
    std_msgs::Float64 azimuth;
    azimuth.data = (float) angle_;
    pub_az_.publish(azimuth);
    
    //Formula for distance calculation in metres
    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::spherical_equatorial<boost::geometry::degree>> spherical_point;
    spherical_point p(sensor_long_, sensor_lat_);
    spherical_point q(goal_long_, goal_lat_);
    double dist = boost::geometry::distance(p, q);
    distance_ = dist*earth_radius*1000;

    //Print the Distance from the goal 
    std::cout<<"distance is "<<distance_<<std::endl;
}

void globalPlanner::rotation()
{                                
    float ang_z,vel_x;                              
    float kp = 0.05, kd = 0.06;                     
    float cp = 0.75;                     

    if(yaw_<0)
        yaw_ = yaw_ + circle;
    if(angle_<0)
        angle_ = angle_ + circle;
    diff_angle = angle_ - yaw_;

    double diff_angle_rad; 

    if(diff_angle < 0)
    {
        if(abs(diff_angle)>semi_circle)
        {
            diff_angle =  diff_angle + circle;
            diff_angle_rad = abs(diff_angle) * M_PI /semi_circle;               
            ang_z = - diff_angle_rad * cp; 
        }
        else
        {
            diff_angle_rad = abs(diff_angle) * M_PI /semi_circle;               
            ang_z = diff_angle_rad * cp;
        }

    }
    else
    {
        if(abs(diff_angle)>semi_circle)
        {
            diff_angle =  circle - diff_angle;
            diff_angle_rad = abs(diff_angle) * M_PI /semi_circle;               
            ang_z = diff_angle_rad * cp;
        }
        else
        {
            diff_angle_rad = abs(diff_angle) * M_PI /semi_circle;               
            ang_z = - diff_angle_rad * cp;                                           
        }
    }

    vel_x = distance_ * kp + kd*(abs(in_distance_ - distance_)) / 0.1;

    if((ang_z > ang_max) && (ang_z > 0))
        ang_z = ang_max;

    if((ang_z < - ang_max) && (ang_z < 0))
        ang_z = - ang_max;



    if(vel_x > linear_max)
        vel_x = linear_max;
    if(vel_x < linear_min)
        vel_x = linear_min;

    if(abs(diff_angle) > ang_thres)
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

void globalPlanner::linearMovement()
{
    float kp = 0.05, kd = 0.06;  
    float vel_x;                                           
    vel_x = distance_ * kp + kd*(abs(in_distance_ - distance_))/0.1;

    if(vel_x > linear_max)
        vel_x = linear_max;
    if(vel_x < linear_min)
        vel_x = linear_min;

    this->vel_.linear.x = vel_x;

    pub_vel_.publish(vel_);

    // if(distance_>dist_thres)
    //     pub_vel_.publish(vel_);

    // else
    // {
    //     vel_.linear.x = 0;
    //     vel_.angular.z = 0;
    //     pub_vel_.publish(vel_);
    //     std::cout<<"Goal Reached";
    //     if(ros::ok)
    //         ros::shutdown();       
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "globalPlanner");
    globalPlanner obj;
    obj.caller();
    return 0;
}
