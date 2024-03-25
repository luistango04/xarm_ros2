#include "xarm_planner/xarm_planner.h"

class PoseSubscriberNode : public rclcpp::Node
{
public:
    PoseSubscriberNode() : Node("pose_subscriber_node")
    {
        // Initialize XArm planner
        xarm_planner_ = std::make_shared<xarm_planner::XArmPlanner>("xarm7"); // Assuming "xarm7" as the group name

        // Subscribe to pose topic
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10, std::bind(&PoseSubscriberNode::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Pose Subscriber Node has started.");
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose_msg)
    {
        // Update the latest received pose
        latest_pose_ = *pose_msg;

        // Plan and execute path for the latest received pose
        xarm_planner_->planPoseTarget(latest_pose_);
        xarm_planner_->executePath();
    }

    std::shared_ptr<xarm_planner::XArmPlanner> xarm_planner_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
    geometry_msgs::msg::Pose latest_pose_; // Variable to store the latest received pose
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<PoseSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}

