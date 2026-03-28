#include <cmath>
#include <algorithm>
#include "../include/my_auto_nav_pkg/astar_planner.hpp"

namespace astar_planner{

    AstarPlanner::AstarPlanner() {}
    AstarPlanner::~AstarPlanner() {}

    void AstarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ){
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
    }

    void AstarPlanner::cleanup(){
        RCLCPP_INFO(node_->get_logger(), "Cleanup the plugin %s of type Astar_planner", name_.c_str());
    }

    void AstarPlanner::activate(){
        RCLCPP_INFO(node_->get_logger(), "Activating the plugin %s of type Astar_planner", name_.c_str());
    }

    void AstarPlanner::deactivate(){
        RCLCPP_INFO(node_->get_logger(), "Deactivating the plugin %s of type Astar_planner", name_.c_str());
    }

    nav_msgs::msg::Path AstarPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal, std::function<bool()> cancel_checker){

        std::vector<std::pair<int,int>> explore_dir = {{1,0},{0,1},{-1,0},{0,-1}};
        std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
        
        if (cancel_checker && cancel_checker()) {
            RCLCPP_WARN(node_->get_logger(), "A* Planning cancelled by navigator!");
            return nav_msgs::msg::Path(); 
        }
        
        unsigned int size_x = costmap_->getSizeInCellsX();
        unsigned int size_y = costmap_->getSizeInCellsY();
        std::vector<bool> visited_nodes(size_x * size_y, false);

        GraphNode start_node = world_to_grid(start.pose);
        GraphNode goal_node = world_to_grid(goal.pose);

        start_node.heuristic = manhattan_dist(start_node, goal_node);
        pending_nodes.push(start_node);

        if (poseOnMap(start_node)) {
            visited_nodes[start_node.y * size_x + start_node.x] = true;
        }

        GraphNode active_node;
        int iterations = 0;
        int max_iterations = 200000; 

        while (!pending_nodes.empty() && rclcpp::ok() && iterations < max_iterations){
            iterations++;
            active_node = pending_nodes.top();
            pending_nodes.pop();

            if (active_node == goal_node){
                break;
            }

            for (const auto &dir : explore_dir) {
                GraphNode new_node = active_node + dir ;
                
                if (!poseOnMap(new_node)) continue;

                unsigned int new_node_idx = new_node.y * size_x + new_node.x;

                if (!visited_nodes[new_node_idx] && costmap_->getCost(new_node.x, new_node.y) < 128){
                    visited_nodes[new_node_idx] = true; 
                    
                    new_node.cost = active_node.cost + 1 + costmap_->getCost(new_node.x, new_node.y);
                    new_node.heuristic = manhattan_dist(new_node, goal_node);
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    
                    pending_nodes.push(new_node);
                }
            }
        }

        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now(); 

        if (active_node == goal_node) {
            while (active_node.prev && rclcpp::ok()) {
                geometry_msgs::msg::Pose last_pose = grid_to_world(active_node);
                geometry_msgs::msg::PoseStamped last_pose_stamped;
                last_pose_stamped.header.frame_id = global_frame_;
                last_pose_stamped.header.stamp = node_->now();
                last_pose_stamped.pose = last_pose;
                active_node = *active_node.prev;
                path.poses.push_back(last_pose_stamped);
            }
            std::reverse(path.poses.begin(), path.poses.end());
            return path;
        } else {
            RCLCPP_WARN(node_->get_logger(), "A* failed to find a valid path! (Target might be blocked)");
            return path; 
        }
    }

    double AstarPlanner::manhattan_dist(const GraphNode &node, const GraphNode &goal_node){
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y);
    }

    GraphNode AstarPlanner::world_to_grid(const geometry_msgs::msg::Pose &pose){
        int grid_x = static_cast<int>((pose.position.x - costmap_-> getOriginX()) / costmap_->getResolution());
        int grid_y = static_cast<int>((pose.position.y - costmap_-> getOriginY()) / costmap_->getResolution());
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AstarPlanner::grid_to_world(const GraphNode &node){
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
        pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
        return pose;
    }

    bool AstarPlanner::poseOnMap(const GraphNode &node){
        return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav2_core::GlobalPlanner)