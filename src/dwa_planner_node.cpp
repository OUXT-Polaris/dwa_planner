// headers for ros
#include <ros/ros.h>

//headers in this package
#include <dwa_planner/dwa_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dwa_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    DwaPlanner planner(nh,pnh);
    ros::spin();
    return 0;
}