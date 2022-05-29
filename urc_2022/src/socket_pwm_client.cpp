#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#define PORT 5005

using namespace std;

float linear = 0;
float angular = 0;
float vel_L = 0;
float vel_R = 0;
int pwm_L = 0;
int pwm_R = 0;
string msg = "";

int sock = 0, client_fd;

string fill(int pwm)
{
    string p = to_string(pwm);
    int len = strlen(p.c_str());   
    p.insert(p.begin(), 3 - len, '0');
    return(p);  
}

void callback(const geometry_msgs::Twist::ConstPtr& data) 
{
    cout<<"hi"<<endl;
    
    int dir_L = 1;
    int dir_R = 1;

    char* msg_f;

    linear = data->linear.x;
    angular = data->angular.z;

    vel_L = linear - angular * 0.35;
    vel_R = linear + angular * 0.35;

    pwm_L = int(vel_L * 255 / 0.975);
    pwm_R = int(vel_R * 255 / 0.975);

    if(abs(pwm_L) > 255)
        pwm_L = 255;
    if(abs(pwm_R)>255)
        pwm_R = 255;

    if(vel_L < 0)
    {
        dir_L = 0;
        pwm_L = int(abs(pwm_L));
    }

    if(vel_R < 0)
    {
        dir_R = 0;
        pwm_R = int(abs(pwm_R));
    }

    msg = "a" + fill(pwm_R) + to_string(dir_R) + fill(pwm_L) + to_string(dir_L);
    msg_f = (char*)malloc(strlen(msg.c_str())*sizeof(char));
    strcpy(msg_f, msg.c_str());
    ROS_INFO("sending %s",msg_f);
    send(sock, msg_f, strlen(msg_f), 0);
}

void caller(ros::NodeHandle nh)
{
	ros::Subscriber client_sub = nh.subscribe("/cmd_vel", 1, callback);
	ros::spin();
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv, "client_node");
	ros::NodeHandle nh;
	//int sock = 0, client_fd;
	struct sockaddr_in serv_addr;

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);

	if (inet_pton(AF_INET, "192.168.1.134", &serv_addr.sin_addr) <= 0) 
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if ((client_fd = connect(sock, (struct sockaddr*)&serv_addr,sizeof(serv_addr))) < 0) 
	{
		printf("\nConnection Failed \n");
		return -1;
	}

	caller(nh);

	close(client_fd);
	return 0;
}


