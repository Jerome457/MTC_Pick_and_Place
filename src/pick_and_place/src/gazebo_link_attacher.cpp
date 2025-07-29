#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#include <cmath>

class GripperMonitor : public rclcpp::Node {
public:
    GripperMonitor()
        : Node("gripper_monitor"),
          last_state(State::UNKNOWN)
    {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GripperMonitor::jointStateCallback, this, std::placeholders::_1)
        );
        attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

        RCLCPP_INFO(this->get_logger(), "GripperMonitor node started.");
    }

private:
    enum class State { UNKNOWN, OPEN, CLOSED };

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;

    State last_state;

    static constexpr double OPEN_VAL = 0.3992;
    static constexpr double CLOSE_VAL = -0.15;
    static constexpr double TOL = 0.0002;

    bool isOpen(double val) const {
        return std::abs(val - OPEN_VAL) <= TOL;
    }
    bool isClosed(double val) const {
        return std::abs(val - CLOSE_VAL) <= TOL;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double joint1_val = 0.0;
        double joint2_val = 0.0;
        bool found1 = false, found2 = false;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "AX_12A_04_Revolute-34") {
                joint1_val = msg->position[i];
                found1 = true;
            } else if (msg->name[i] == "link6_Revolute-24") {
                joint2_val = msg->position[i];
                found2 = true;
            }
        }

        if (!found1 && !found2) return; // use either joint as needed

        State current_state = State::UNKNOWN;
        if ((found1 && isOpen(joint1_val)) || (found2 && isOpen(joint2_val)))
            current_state = State::OPEN;
        else if ((found1 && isClosed(joint1_val)) || (found2 && isClosed(joint2_val)))
            current_state = State::CLOSED;
        else
            return; // Not clearly open or closed

        if (last_state == State::UNKNOWN) {
            last_state = current_state;
            return;
        }

        if (current_state == last_state) return;

        if (current_state == State::CLOSED && last_state == State::OPEN) {
            RCLCPP_INFO(this->get_logger(), "Transition open→closed: ATTACH.");
            callAttachService();
        }
        else if (current_state == State::OPEN && last_state == State::CLOSED) {
            RCLCPP_INFO(this->get_logger(), "Transition closed→open: DETACH.");
            callDetachService();
        }

        last_state = current_state;
    }

    void callAttachService() {
        if (!attach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "AttachLink service not available.");
            return;
        }

        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "arm_urdf";
        request->link1_name = "Gear_Flange_Motor";
        request->model2_name = "cylinder";
        request->link2_name = "cylinder_link";

        auto future = attach_client_->async_send_request(
            request,
            [this](rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedFuture result) {
                if (result.valid()) {
                    RCLCPP_INFO(this->get_logger(),
                        "AttachLink response: success = %s",
                        result.get()->success ? "true" : "false");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call AttachLink service.");
                }
            }
        );
    }

    void callDetachService() {
        if (!detach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "DetachLink service not available.");
            return;
        }

        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "arm_urdf";
        request->link1_name = "Gear_Flange_Motor";
        request->model2_name = "cylinder";
        request->link2_name = "cylinder_link";

        auto future = detach_client_->async_send_request(
            request,
            [this](rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedFuture result) {
                if (result.valid()) {
                    RCLCPP_INFO(this->get_logger(),
                        "DetachLink response: success = %s",
                        result.get()->success ? "true" : "false");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to call DetachLink service.");
                }
            }
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperMonitor>());
    rclcpp::shutdown();
    return 0;
}
