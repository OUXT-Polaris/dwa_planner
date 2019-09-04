#ifndef DWA_PLANNER_DWA_PLANNER_H_INCLUDED
#define DWA_PLANNER_DWA_PLANNER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <usv_navigation_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <quaternion_operation/quaternion_operation.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

// Headers in STL
#include <memory>

// Headers in this package
#include <dwa_planner/DwaPlannerConfig.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> SyncPolicy;

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
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    void poseTwistCallback(const geometry_msgs::PoseStamped::ConstPtr pose,const geometry_msgs::TwistStamped::ConstPtr twist);
    dynamic_reconfigure::Server<dwa_planner::DwaPlannerConfig> server_;
    dynamic_reconfigure::Server<dwa_planner::DwaPlannerConfig>::CallbackType param_func_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    usv_navigation_msgs::Path path_;
    std::string twist_stamped_topic_;
    ros::Subscriber path_sub_;
    std::string path_topic_;
    ros::Publisher twist_cmd_pub_;
    ros::Publisher marker_pub_;
    std::string twist_cmd_topic_;
    std::string current_pose_topic_;
    std::string robot_frame_;
    std::vector<double> getAngularVelList(geometry_msgs::TwistStamped twist);
    std::vector<double> getLinearVelList(geometry_msgs::TwistStamped twist);
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped> > pose_sub_ptr_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped> > twist_sub_ptr_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_ptr_;
    std::vector<geometry_msgs::PoseStamped> predictPath(double linear_vel,double angular_vel);
    visualization_msgs::MarkerArray generateMarker(std::vector<std::vector<geometry_msgs::PoseStamped> > paths,ros::Time header);
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::TransformStamped transform_stamped_;
};

#endif  //DWA_PLANNER_DWA_PLANNER_H_INCLUDED