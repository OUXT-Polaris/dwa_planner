//headers in this package
#include <dwa_planner/dwa_planner.h>

DwaPlanner::DwaPlanner(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    param_func_ = boost::bind(&DwaPlanner::paramsCallback, this, _1, _2);
    server_.setCallback(param_func_);
    pnh_.param<std::string>("twist_stamped_topic", twist_stamped_topic_, ros::this_node::getName()+"/twist_stamped");
    pnh_.param<std::string>("path_topic", path_topic_, ros::this_node::getName()+"/path");
    pnh_.param<std::string>("twist_cmd_topic", twist_cmd_topic_, ros::this_node::getName()+"/twist_cmd_topic");
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_cmd_topic_,1);
    twist_stamped_sub_ = nh_.subscribe(twist_stamped_topic_,1,&DwaPlanner::twistStampedCallback,this);
    path_sub_ = nh_.subscribe(path_topic_,1,&DwaPlanner::pathCallback,this);
}

DwaPlanner::~DwaPlanner()
{

}

void DwaPlanner::paramsCallback(dwa_planner::DwaPlannerConfig &config, uint32_t level)
{
    cpnfig_ = config;
    return;
}

void DwaPlanner::pathCallback(const usv_navigation_msgs::Path::ConstPtr msg)
{
    path_ = *msg;
    return;
}

void DwaPlanner::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr msg)
{
    return;
}