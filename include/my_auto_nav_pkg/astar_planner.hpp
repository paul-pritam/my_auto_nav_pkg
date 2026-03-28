#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <string>
#include <memory>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace astar_planner{
    struct GraphNode{
        int x;
        int y;
        int cost;
        double heuristic;
        std::shared_ptr<GraphNode> prev;

        GraphNode() : GraphNode(0,0) {}
        GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0){}
        
        bool operator > (const GraphNode other) const{
            return cost+heuristic > other.cost + other.heuristic;
        }

        bool operator == (const GraphNode &other) const{
            return x == other.x && y == other.y;
        }

        GraphNode operator + (std::pair<int, int> const &other) {
            GraphNode res(x + other.first, y + other.second);
            return res;
        }       
    };

    class AstarPlanner : public nav2_core::GlobalPlanner{
        public:
            AstarPlanner();
            ~AstarPlanner();

            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros //builds the map
            ) override;

            void cleanup() override;
            void activate() override;
            void deactivate() override;

            nav_msgs::msg::Path createPlan(
                const geometry_msgs::msg::PoseStamped &start,
                const geometry_msgs::msg::PoseStamped &goal,
                std::function<bool()> cancel_checker
            )override;
        
        private:
            std::shared_ptr<tf2_ros::Buffer> tf_;
            nav2_util::LifecycleNode::SharedPtr node_;
            nav2_costmap_2d::Costmap2D *costmap_; //result map for algo
            std::string global_frame_, name_;
            double interpolation_res_;

            bool poseOnMap(const GraphNode &node);
            GraphNode world_to_grid(const geometry_msgs::msg::Pose &pose);
            geometry_msgs::msg::Pose grid_to_world(const GraphNode &node);
            double manhattan_dist(const GraphNode &present_node , const GraphNode &goal_node);
    };
}
#endif