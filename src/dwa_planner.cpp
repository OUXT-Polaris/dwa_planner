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
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, ros::this_node::getName()+"/current_pose_topic");
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_cmd_topic_,1);
    twist_sub_ptr_ = std::make_shared<message_filters::Subscriber<geometry_msgs::TwistStamped> >(nh_,twist_stamped_topic_,10);
    pose_sub_ptr_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped> >(nh_,current_pose_topic_,10);
    sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10), *pose_sub_ptr_, *twist_sub_ptr_ );
    sync_ptr_->registerCallback(boost::bind(&DwaPlanner::poseTwistCallback, this, _1, _2));
    path_sub_ = nh_.subscribe(path_topic_,1,&DwaPlanner::pathCallback,this);
}

DwaPlanner::~DwaPlanner()
{

}

void DwaPlanner::poseTwistCallback(const geometry_msgs::PoseStamped::ConstPtr pose,const geometry_msgs::TwistStamped::ConstPtr twist)
{
    std::vector<double> angular_vel_list = getAngularVelList(*twist);
    std::vector<double> linear_vel_list = getAngularVelList(*twist);
    return;
}

void DwaPlanner::paramsCallback(dwa_planner::DwaPlannerConfig &config, uint32_t level)
{
    config_ = config;
    return;
}

void DwaPlanner::pathCallback(const usv_navigation_msgs::Path::ConstPtr msg)
{
    path_ = *msg;
    return;
}

std::vector<double> DwaPlanner::getAngularVelList(geometry_msgs::TwistStamped twist)
{
    std::vector<double> ret;
    double angular_vel  = twist.twist.angular.z;
    ret.push_back(angular_vel);
    while(true)
    {
        angular_vel = angular_vel + config_.lim_angular_acc*config_.sampling_time;
        if(angular_vel>config_.lim_angular_vel)
        {
            break;
        }
        else
        {
            ret.push_back(angular_vel);
        }
    }
    while(true)
    {
        angular_vel = angular_vel - config_.lim_angular_acc*config_.sampling_time;
        if(angular_vel<(config_.lim_angular_vel)*-1)
        {
            break;
        }
        else
        {
            ret.push_back(angular_vel);
        }
    }
    return ret;
}

std::vector<double> DwaPlanner::getLinearVelList(geometry_msgs::TwistStamped twist)
{
    std::vector<double> ret;
    double linear_vel  = twist.twist.linear.z;
    ret.push_back(linear_vel);
    while(true)
    {
        linear_vel = linear_vel + config_.lim_linear_acc*config_.sampling_time;
        if(linear_vel>config_.lim_linear_vel)
        {
            break;
        }
        else
        {
            ret.push_back(linear_vel);
        }
    }
    while(true)
    {
        linear_vel = linear_vel - config_.lim_linear_acc*config_.sampling_time;
        if(linear_vel<(config_.lim_linear_vel)*-1)
        {
            break;
        }
        else
        {
            ret.push_back(linear_vel);
        }
    }
    return ret;
}