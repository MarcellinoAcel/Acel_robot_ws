#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <icecream.hpp>
#include "bv_nav/convertion.hpp"
using std::placeholders::_1;

class Bv_nav : public rclcpp::Node
{

public:
    Bv_nav() : Node("bv_nav")
    {

        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        this->subscription_button_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "button", 10, std::bind(&Bv_nav::sign_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /button");

        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        pub_pose = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_position", 10);

        sub_amcl = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&Bv_nav::robot_pose, this, _1));
    }

    void send_goal(double x, double y, double theta)
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.z = theta; // Set quaternion based on your needs
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&Bv_nav::goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&Bv_nav::get_result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void robot_pose(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        Convertion::Quaternion q = {
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        };
        double yaw, pitch, roll;
        conv.quat_to_eular(q, yaw, pitch, roll);

        auto robot_pose = geometry_msgs::msg::Pose2D();
        robot_pose.x = msg.pose.pose.position.x;
        robot_pose.y = msg.pose.pose.position.y;
        robot_pose.theta = conv.toDeg(yaw);

        pub_pose->publish(robot_pose);

        // IC(msg.pose.pose.position.x, msg.pose.pose.position.y, conv.toDeg(yaw));
    }

    void sign_callback(const std_msgs::msg::Int32MultiArray &msg)
    {
        button but;
        but.A = msg.data[0];
        but.Y = msg.data[4];
        but.X = msg.data[3];

        if (but.X)
        {
            client_ptr_->async_cancel_all_goals();
        }
        else if (but.A)
        {
            send_goal(0, 0, 0);
        }
        else if (but.Y)
        {
            send_goal(3, 2, 0);
        }
    }

private:
    struct button
    {
        int A;
        int Y;
        int B;
        int X;
        int Up;
        int Down;
        int Left;
        int home;
        int Right;
        int start;
        int select;
        int RT;
        int RB;
        int LB;
        int LT;
    };
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_button;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl;

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalHandleNavigateToPose::SharedPtr goal_handle_;
    Convertion conv;

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_button_;

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void get_result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Bv_nav>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in node: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <std_msgs/msg/int32_multi_array.hpp>
// #include <memory>
// #include <functional>

// class NavigateToPoseClient : public rclcpp::Node
// {
// public:
//     using NavigateToPose = nav2_msgs::action::NavigateToPose;
//     using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

//     NavigateToPoseClient() : Node("navigate_to_pose_client")
//     {
//         this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
//             this,
//             "navigate_to_pose");

//         this->subscription_button_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
//             "button", 10, std::bind(&NavigateToPoseClient::sign_callback, this, std::placeholders::_1));

//         RCLCPP_INFO(this->get_logger(), "Subscribed to /button");
//     }

//     void send_goal(double x, double y, double theta)
//     {
//         using namespace std::placeholders;

//         if (!this->client_ptr_->wait_for_action_server()) {
//             RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
//             return;
//         }

//         auto goal_msg = NavigateToPose::Goal();
//         goal_msg.pose.pose.position.x = x;
//         goal_msg.pose.pose.position.y = y;
//         goal_msg.pose.pose.orientation.z = theta;  // Set quaternion based on your needs
//         goal_msg.pose.header.frame_id = "map";

//         RCLCPP_INFO(this->get_logger(), "Sending goal");

//         auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
//         send_goal_options.goal_response_callback =
//             std::bind(&NavigateToPoseClient::goal_response_callback, this, _1);
//         send_goal_options.result_callback =
//             std::bind(&NavigateToPoseClient::get_result_callback, this, _1);
//         this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
//     }

// private:
//     rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
//     rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_button_;

//     void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
//     {
//         if (!goal_handle) {
//             RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//         }
//     }

//     void get_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
//     {
//         switch (result.code) {
//             case rclcpp_action::ResultCode::SUCCEEDED:
//                 RCLCPP_INFO(this->get_logger(), "Goal succeeded");
//                 break;
//             case rclcpp_action::ResultCode::ABORTED:
//                 RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//                 return;
//             case rclcpp_action::ResultCode::CANCELED:
//                 RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//                 return;
//             default:
//                 RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//                 return;
//         }
//     }

//     void sign_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
//     {
//         int button_A = msg->data[0];
//         int button_Y = msg->data[4];
//         int button_X = msg->data[3];

//         if (button_A) {
//             RCLCPP_INFO(this->get_logger(), "Button 6 pressed, moving to {0, 0, 0}");
//             send_goal(0.0, 0.0, 0.0);
//         } else if (button_Y) {
//             RCLCPP_INFO(this->get_logger(), "Button Y pressed, moving to {2, 0, 0}");
//             send_goal(2.0, 0.0, 0.0);
//         } else if (button_X) {
//             RCLCPP_INFO(this->get_logger(), "Cancel journey");
//             // Implement cancel_goal() functionality here
//         }
//     }
// };

// int main(int argc, char ** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<NavigateToPoseClient>();

//     try {
//         rclcpp::spin(node);
//     } catch (const std::exception & e) {
//         RCLCPP_ERROR(node->get_logger(), "Exception in node: %s", e.what());
//     }

//     rclcpp::shutdown();
//     return 0;
// }
