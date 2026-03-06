#include <algorithm>    

#include "nav2_util/node_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "../include/my_auto_nav_pkg/pure_pursuit.hpp"

namespace pure_pursuit{

    PurePursuit::PurePursuit() {}
    PurePursuit::~PurePursuit() {}

    void PurePursuit::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
            ){

                node_ =parent;

                auto node = node_.lock();

                costmap_ros_ = costmap_ros;
                tf_buffer_ = tf_buffer;
                plugin_name_ = name;
                logger_ = node->get_logger();
                clock_ = node->get_clock();

                nav2_util::declare_parameter_if_not_declared(node, plugin_name_+".la_dist", rclcpp::ParameterValue(0.5));
                nav2_util::declare_parameter_if_not_declared(node, plugin_name_+".max_linear_vel", rclcpp::ParameterValue(0.3));
                nav2_util::declare_parameter_if_not_declared(node, plugin_name_+".max_angular_vel", rclcpp::ParameterValue(1.0));

                node->get_parameter(plugin_name_+".la_dist",la_dist_);
                node->get_parameter(plugin_name_+".max_linear_vel",max_linear_vel_);
                node->get_parameter(plugin_name_+".max_angular_vel",max_angular_vel_);

                la_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pure_pursuit/look_ahead_pose", 1);

            }

    void PurePursuit::cleanup(){
        RCLCPP_INFO(logger_, "cleaning up pure pure pursuit");
        la_pub_.reset();
    }

    void PurePursuit::activate(){
        RCLCPP_INFO(logger_, "activating pure pursuit");
        la_pub_->on_activate();
    }

    void PurePursuit::deactivate(){
        RCLCPP_INFO(logger_, "deactivating pure pursuit");
        la_pub_->on_deactivate();
    }

    geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &robot_pose,
        const geometry_msgs::msg::Twist &,
        nav2_core::GoalChecker *
    ){
        auto node = node_.lock();
        geometry_msgs::msg::TwistStamped vel_pub;
        vel_pub.header.frame_id = robot_pose.header.frame_id;

        if (global_plan_.poses.empty()){
            RCLCPP_ERROR(logger_, "Empty global path plan");
            return vel_pub;
        }

        if (!transform_plan(robot_pose.header.frame_id)){
            RCLCPP_ERROR(logger_, "unable to transform global path plan in the bot_pose_tf");
            return vel_pub;
        }

        auto la_pose = getLookaheadPose(robot_pose);
        la_pub_->publish(la_pose);

        double dx_goal = global_plan_.poses.back().pose.position.x - robot_pose.pose.position.x;
        double dy_goal = global_plan_.poses.back().pose.position.y - robot_pose.pose.position.y;
        double distance_to_goal = std::sqrt(dx_goal*dx_goal + dy_goal*dy_goal);

        //calcuate curvature
        tf2::Transform bot_to_look_ahead_pose_tf, w_to_robot_tf, w_to_lookahead_pose_tf;

        
        tf2::fromMsg(robot_pose.pose, w_to_robot_tf);
        tf2::fromMsg(la_pose.pose, w_to_lookahead_pose_tf);

        
        bot_to_look_ahead_pose_tf = w_to_robot_tf.inverse() * w_to_lookahead_pose_tf;
        tf2::toMsg(bot_to_look_ahead_pose_tf, la_pose.pose);

       
        double curvature = get_curvature(la_pose);

        double desired_linear_vel = max_linear_vel_;
        if (distance_to_goal < la_dist_) {
            //deaccelerate the robot
            desired_linear_vel = max_linear_vel_ * (distance_to_goal / la_dist_);
            desired_linear_vel = std::max(0.05, desired_linear_vel); 
        }            
        
        vel_pub.twist.linear.x = max_linear_vel_;
        vel_pub.twist.angular.z = curvature * max_angular_vel_;

        return vel_pub;
    }

    bool PurePursuit::transform_plan(const std::string &frame){
        if(global_plan_.header.frame_id==frame){
            return true;
        }
        geometry_msgs::msg::TransformStamped transform;
        try{
            transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
        }catch(tf2::ExtrapolationException &ex){
            RCLCPP_ERROR_STREAM(logger_, "couldnt transform plan to frame"<<global_plan_.header.frame_id<<"to"<<frame);
            return false;
        }
        for (auto &pose : global_plan_.poses){
            tf2::doTransform(pose,pose, transform);
        }
        global_plan_.header.frame_id = frame;
        return true;
    }

    void PurePursuit::setPlan(const nav_msgs::msg::Path &path){
        RCLCPP_INFO_STREAM(logger_, "path received with " << path.poses.size()<<"poses");
        RCLCPP_INFO_STREAM(logger_, "Path frame" << path.header.frame_id);
        global_plan_ = path;
    }

    void PurePursuit::setSpeedLimit(const double &, const bool &){}

    double PurePursuit::get_curvature(const geometry_msgs::msg::PoseStamped &la_pose){
        const double la_distance = (la_pose.pose.position.x * la_pose.pose.position.x) + (la_pose.pose.position.y * la_pose.pose.position.y);

        if (la_distance > 0.001){
            return 2.0 * la_pose.pose.position.y / la_distance;
        }
        else {
            return 0.0;
        }
    }

    geometry_msgs::msg::PoseStamped PurePursuit::getLookaheadPose(const geometry_msgs::msg::PoseStamped &robot_pose){

        geometry_msgs::msg::PoseStamped la_pose = global_plan_.poses.back();
        for  (auto pose_it = global_plan_.poses.rbegin(); pose_it!=global_plan_.poses.rend(); ++pose_it){
            double dx = pose_it -> pose.position.x - robot_pose.pose.position.x;
            double dy = pose_it -> pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            if(distance>la_dist_){
                la_pose = *pose_it;
            }
            else{
                break;
            }
        }
        return la_pose;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pure_pursuit::PurePursuit, nav2_core::Controller)