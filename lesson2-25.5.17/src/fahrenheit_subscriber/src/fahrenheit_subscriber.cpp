#include "rclcpp/rclcpp.hpp"
#include "fahrenheit_interfaces/msg/fahrenheit.hpp"
#include "fahrenheit_interfaces/msg/celsius.hpp"
#include <chrono>
#include <string>

using namespace std::chrono_literals;
//using namespace fahrenhrit_interfaces::msg;

class TemperatureTransfer : public rclcpp::Node
{
    private:
        rclcpp::Subscription<fahrenheit_interfaces::msg::Fahrenheit>::SharedPtr subscriber_;
        rclcpp::Publisher<fahrenheit_interfaces::msg::Fahrenheit>::SharedPtr publisher_;
        float received_temperature_;
    
    public:
        explicit TemperatureTransfer(const std::string &node_name):Node(node_name)
        {
            
            publisher_ = this->create_publisher<fahrenheit_interfaces::msg::Fahrenheit>("Celsius", 10);
            subscriber_ = this->create_subscription<fahrenheit_interfaces::msg::Fahrenheit>("Fahrenheit",10,
            [&](const fahrenheit_interfaces::msg::Fahrenheit::SharedPtr msg)
            ->void{
                //RCLCPP_INFO(this->get_logger(),"Subscriber available");
                RCLCPP_INFO(this->get_logger(),"Fahrenheit_Received: '%f' F",msg->fahrenheit);
                received_temperature_ = msg->fahrenheit;
                auto celsius_msg = fahrenheit_interfaces::msg::Fahrenheit();

                if(received_temperature_!=0)celsius_msg.fahrenheit= (received_temperature_ - 32) * 5.0 / 9.0;

                RCLCPP_INFO(this->get_logger(),"Celsius_Publishing: '%f' C",celsius_msg.fahrenheit);
                publisher_->publish(celsius_msg);
            });
        }

};


int main(int argc,char **argv){
  
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TemperatureTransfer>("temperature");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
