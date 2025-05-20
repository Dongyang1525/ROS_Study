#include "rclcpp/rclcpp.hpp"
#include "fahrenheit_interfaces/msg/fahrenheit.hpp"
#include <random>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class FahrenheitPublisher : public rclcpp::Node
{
private:
    
    rclcpp::Publisher<fahrenheit_interfaces::msg::Fahrenheit>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    explicit FahrenheitPublisher(const std::string &node_name):Node(node_name)
    {
        publisher_ =this->create_publisher<fahrenheit_interfaces::msg::Fahrenheit>("Fahrenheit",10);
        timer_ = this->create_wall_timer(
            1000ms,
            std::bind(&FahrenheitPublisher::timer_callback,this)
        );
        
    }

    void timer_callback()
    {
        auto msg = fahrenheit_interfaces::msg::Fahrenheit();
        msg.fahrenheit = generate_random_temperature();
        RCLCPP_INFO(this->get_logger(),"Fahrenheit_Publishing: '%f' F",msg.fahrenheit);
        publisher_->publish(msg);
    }

    float generate_random_temperature()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0.0, 100.0);
        return dis(gen);
    } 
};

int main(int argc,char **argv){
  
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FahrenheitPublisher>("temperature");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}



