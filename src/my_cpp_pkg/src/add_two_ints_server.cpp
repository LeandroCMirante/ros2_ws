#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
using namespace std::placeholders;
 
class AddTwoIntsServerNode : public rclcpp::Node 
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server") 
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2)
        );
        RCLCPP_INFO(this->get_logger(), "Service 'add_two_ints' is ready.");
    }
 
private:
    void callbackAddTwoInts(
        const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
        example_interfaces::srv::AddTwoInts::Response::SharedPtr response
    ) 
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Sending response: sum=%ld", response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}