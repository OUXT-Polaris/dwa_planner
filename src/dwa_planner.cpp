//headers in this package
#include <dwa_planner/dwa_planner.h>

DwaPlanner::DwaPlanner(ros::NodeHandle nh,ros::NodeHandle pnh) : tf_listener_(tf_buffer_)
{
    nh_ = nh;
    pnh_ = pnh;
    param_func_ = boost::bind(&DwaPlanner::paramsCallback, this, _1, _2);
    server_.setCallback(param_func_);
    pnh_.param<std::string>("twist_stamped_topic", twist_stamped_topic_, ros::this_node::getName()+"/twist_stamped");
    pnh_.param<std::string>("path_topic", path_topic_, ros::this_node::getName()+"/path");
    pnh_.param<std::string>("twist_cmd_topic", twist_cmd_topic_, ros::this_node::getName()+"/twist_cmd_topic");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, ros::this_node::getName()+"/current_pose_topic");
    pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
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
    std::vector<double> linear_vel_list = getLinearVelList(*twist);
    try
    {
        transform_stamped_ = tf_buffer_.lookupTransform(pose->header.frame_id, robot_frame_, pose->header.stamp);
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }
    std::vector<std::vector<geometry_msgs::PoseStamped> > paths;
    for(auto angular_vel_itr = angular_vel_list.begin(); angular_vel_itr != angular_vel_list.end(); angular_vel_itr++)
    {
        for(auto linear_vel_itr = linear_vel_list.begin(); linear_vel_itr != linear_vel_list.end(); linear_vel_itr++)
        {
            std::vector<geometry_msgs::PoseStamped> path = predictPath(*linear_vel_itr,*angular_vel_itr);
            paths.push_back(path);
        }
    }
    visualization_msgs::MarkerArray marker_msg = generateMarker(paths,pose->header.stamp);
    marker_pub_.publish(marker_msg);
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

visualization_msgs::MarkerArray DwaPlanner::generateMarker(std::vector<std::vector<geometry_msgs::PoseStamped> > paths,ros::Time stamp)
{
    visualization_msgs::MarkerArray marker_msg;
    int id = 0;
    for(auto path_itr = paths.begin(); path_itr != paths.end(); path_itr++)
    {
        visualization_msgs::Marker line_marker;
        line_marker.header.stamp = stamp;
        line_marker.header.frame_id = robot_frame_;
        line_marker.ns = "path_line";
        line_marker.id = id;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.frame_locked = true;
        std_msgs::ColorRGBA color;
        color.r = 0.3;
        color.g = 1.0;
        color.b = 0.3;
        color.a = 1.0;
        line_marker.color = color;
        line_marker.scale.x = 0.1;
        line_marker.scale.y = 0.1;
        line_marker.scale.z = 0.1;
        line_marker.lifetime = ros::Duration(1.0);
        for(auto pose_itr = path_itr->begin(); pose_itr != path_itr->end(); pose_itr++)
        {
            geometry_msgs::Point p = pose_itr->pose.position;
            line_marker.points.push_back(p);
            line_marker.colors.push_back(color);
        }
        marker_msg.markers.push_back(line_marker);
        id++;
    }
    return marker_msg;
}

std::vector<geometry_msgs::PoseStamped> DwaPlanner::predictPath(double linear_vel,double angular_vel)
{
    std::vector<geometry_msgs::PoseStamped> ret;
    geometry_msgs::Quaternion quat;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = 1.0;
    geometry_msgs::Point point;
    for(int i=0; i<config_.num_prediction_steps; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = robot_frame_;
        pose.header.stamp = transform_stamped_.header.stamp;
        pose.pose.position = point;
        pose.pose.orientation = quat;
        ret.push_back(pose);
        geometry_msgs::Vector3 rot;
        rot.z = angular_vel*config_.sampling_time;
        quat = quat*quaternion_operation::convertEulerAngleToQuaternion(rot);
        rot = quaternion_operation::convertQuaternionToEulerAngle(quat);
        point.x = point.x + linear_vel*config_.sampling_time*std::cos(rot.z);
        point.y = point.y + linear_vel*config_.sampling_time*std::sin(rot.z);
        //ROS_ERROR_STREAM(point.y);
        //ROS_ERROR_STREAM(linear_vel << "," << linear_vel*config_.sampling_time*std::cos(rot.z));
    }
    return ret;
}

std::vector<double> DwaPlanner::getAngularVelList(geometry_msgs::TwistStamped twist)
{
    std::vector<double> ret;
    double angular_vel  = twist.twist.angular.z;
    ret.push_back(angular_vel);
    while(true)
    {
        angular_vel = angular_vel + config_.delta_angular_vel;
        if(angular_vel>config_.lim_angular_vel || std::fabs(angular_vel-twist.twist.angular.z)>(config_.lim_angular_acc*config_.sampling_time))
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
        angular_vel = angular_vel - config_.delta_angular_vel;
        if(angular_vel<(config_.lim_angular_vel)*-1 || std::fabs(angular_vel-twist.twist.angular.z)>(config_.lim_angular_acc*config_.sampling_time))
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
    double linear_vel  = twist.twist.linear.x;
    ret.push_back(linear_vel);
    while(true)
    {
        linear_vel = linear_vel + config_.delta_linear_vel;
        if(linear_vel>config_.lim_linear_vel || std::fabs(linear_vel-twist.twist.linear.x)>(config_.lim_linear_acc*config_.sampling_time))
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
        linear_vel = linear_vel - config_.delta_linear_vel;
        if(linear_vel<(config_.lim_linear_vel)*-1 || std::fabs(linear_vel-twist.twist.linear.x)>(config_.lim_linear_acc*config_.sampling_time))
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