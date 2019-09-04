#ifndef DWA_PLANNER_DWA_PLANNER_H_INCLUDED
#define DWA_PLANNER_DWA_PLANNER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <usv_navigation_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>

// Headers in this package
#include <dwa_planner/DwaPlannerConfig.h>

class DwaPlanner
{
public:
    DwaPlanner(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~DwaPlanner();
private:
    void paramsCallback(dwa_planner::DwaPlannerConfig &config, uint32_t level);
    dwa_planner::DwaPlannerConfig config_;
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg);
    void pathCallback(const usv_navigation_msgs::Path::ConstPtr msg);
    dynamic_reconfigure::Server<dwa_planner::DwaPlannerConfig> server_;
    dynamic_reconfigure::Server<dwa_planner::DwaPlannerConfig>::CallbackType param_func_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    usv_navigation_msgs::Path path_;
    ros::Subscriber twist_stamped_sub_;
    std::string twist_stamped_topic_;
    ros::Subscriber path_sub_;
    std::string path_topic_;
    ros::Publisher twist_cmd_pub_;
    std::string twist_cmd_topic_;
    std::vector<double> getAngularVelList(geometry_msgs::TwistStamped twist);
    std::vector<double> getLinearVelList(geometry_msgs::TwistStamped twist);
};

#endif  //DWA_PLANNER_DWA_PLANNER_H_INCLUDED