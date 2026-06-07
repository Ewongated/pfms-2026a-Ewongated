#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals; // Needed in the 1s wait for future

// This code is inspired by
// - The setup of the problem presented in : https://foxglove.dev/blog/creating-ros2-services
// - The implementation of service client in a C++ class : https://get-help.theconstruct.ai/t/ros2-service-client-in-c-with-classes-with-node-inheritance/19647
// - Code combined into a class Diagnostic inspired from action client c++ code 

class Diagnostics : public rclcpp::Node
{
public:
Diagnostics() :
    Node("diagnostics_node")
{
  client_ = this->create_client<std_srvs::srv::Trigger>("/checks");  

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&Diagnostics::timer_callback, this));

}

~Diagnostics(){
}

// Here we call the service to check the motors on a timer, but this could be anywhere in the code
// We could call this when we need an update of results from the service
void timer_callback(){
  
    // Wait for the service to be activated
  while (!client_->wait_for_service(1s)) {
    // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    // Print in the screen some information so the user knows what is happening
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Create the request, which is empty
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  service_done_ = false; 

  // Send the request to the service, create a callback to process the response
  auto result_future = client_->async_send_request(
      request, std::bind(&Diagnostics::response_callback, this,
                          std::placeholders::_1));
}

void response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
  // This will keep the response in a loop until the service is done
   while(!service_done_){
        // Check if the future has a valid response, wait for 1 second
        auto status = future.wait_for(1s);
        // If the future has a valid response, print the result, and set the service_done_ to true
        // We compare the status with the ready status
        if (status == std::future_status::ready) {
          // At the moment we are simply printing the results, but we could also use this to
          // update values in the class or do some other processing
          RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                      future.get()->success, future.get()->message.c_str());
          service_done_ = true;
        } 
        else {
          RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    }
}

bool is_service_done() const {
  // Return the service_done_ variable
  return this->service_done_;
}

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false; 

};




int main(int argc, char **argv) {


  // Initiate the rclcpp library
  rclcpp::init(argc, argv);


  auto service_client = std::make_shared<Diagnostics>();

  // Until the service is done, keep spinning
  // spin_some is a non-blocking function that will return immediately if there are no pending work
  // Otherwise, it will execute the work and return, which is triggering the timer callback 
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();

  return 0;
}  
