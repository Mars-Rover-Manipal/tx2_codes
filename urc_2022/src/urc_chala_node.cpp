#include "urc_2022/planners.h"
#include "urc_2022/obstacle.h"

using namespace obstacle_p;
using namespace traversal;

int main(int argc, char** argv)
{
    ros::NodeHandle nh;
    std::string x, y; 
    std::string goal_no;
    nh.setParam("/goalReached", 0);
    nh.setParam("/arucoDetected", 0);
    ros::Publisher pub_vel_;
    ros::init(argc, argv, "URC_ALL_NODE");
    planners obj(nh,pub_vel_);
    obstacle obj1(nh,pub_vel_);
    int flag = 0;
    
    while(ros::ok())
    {
    	nh.getParam("/arucoDetected", y);
    	obj1.caller_obs();
    	if(obj1.samples_[0]>obj1.threshold && obj1.samples_[1]>obj1.threshold && obj1.samples_[2]>obj1.threshold)
    		obj1.decision();
    	else if((flag==0))
    	{
    		nh.getParam("/goalReached", x);
    		if(strcmp(x.c_str(), "1"))
    		{
	    		std::cout<<"Enter goal_no: \n";
	    		std::cin>>goal_no;
	    	}
	    	if(strcmp(y.c_str(), "0"))
    			obj.caller(goal_no);
    	}	
    	//else
    	//{
    		//flag = 1;
    	//}
    }
    return 0;
}



    /*
     * int count = 1;
     * while (1){
     *	
     *	obstacle_avoidance()
     *
     *	switch(count){
     *	
     *	case 1: {
     *		bool reached = globalPlanner(coord[1]);
     *		// flag
     *		if(reached){
     *			bool detected = arucoDetection();
     *			if(detected)
     *				leg_traversal
     *		}
     *		count++;
     *		break;
     *		}
     *  same for 2 and 3
     *
     *  case 4: { 
     *          bool reached = globalPlanner(coord[4]);
     *          if(reached){
     *          	search_pattern();
     *                  bool detected = arucoDetection();
     *                  if(detected)
     *                          leg_traversal();
     *          }
     *
     *          break;
     *          }
     * same for 5
     *  case 6: {
     *  	bool reached = globalPlanner(coord[6]);
     *          if(reached){
     *                  search_pattern();
     *                  bool detected = arucoDetection();
     *                  if(detected)
     *                          gate_traversal();
     *          }
     *		break;
     *		}
     *	}
     *
     *	same for 7
     * }
     * */
    
   
