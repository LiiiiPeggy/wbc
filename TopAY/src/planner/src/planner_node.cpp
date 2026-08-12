#include "planner/planner.h"
#include <ros/ros.h>

using namespace nmoma_planner;

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle root_nh;

    Planner planner;
    
    // ################################
    // C++: Pass the root handle for the global /moma profile
    // ################################
    planner.init(nh, root_nh);

    // ros::MultiThreadedSpinner spinner(2);
    // spinner.spin();

    ros::spin();

    return 0;
}