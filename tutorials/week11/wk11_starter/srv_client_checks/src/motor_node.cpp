// Include the rclcpp library and trigger service type
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <vector>
#include <cstdlib> // For rand()
#include <ctime>   // For seeding rand()

class Motors : public rclcpp::Node {
public:
  Motors() : Node("motor_node") {
    // Seed the random number generator
    std::srand(std::time(nullptr));

    // Declare and get parameters
    motor_count_ = this->declare_parameter<int>("motor_count", 3); // Default to 3 motors
    min_position_ = this->declare_parameter<int>("min_position", 0); // Default min position
    max_position_ = this->declare_parameter<int>("max_position", 5); // Default max position

    // Initialize motor positions vector to between min_position and max_position
    motor_positions_.resize(motor_count_);
    for (int i = 0; i < motor_count_; i++) {
      motor_positions_[i] = min_position_ + (std::rand() % (max_position_ - min_position_ + 1));
    }

    // Initialize motor directions vector with random boolean values
    motor_directions_.resize(motor_count_);
    for (int i = 0; i < motor_count_; i++) {
      motor_directions_[i] = (std::rand() % 2 == 0); // Randomly set to true (forward) or false (inverse)
      checkOrientations(i); // Check orientations to set initial direction
    }

    // Create the "checks" service with a doChecks callback
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "checks", std::bind(&Motors::doChecks, this, std::placeholders::_1, std::placeholders::_2));


    RCLCPP_INFO(this->get_logger(), "Ready to check motors");
    RCLCPP_INFO_STREAM(this->get_logger(), "Min: " << min_position_ << " Max: " << max_position_);
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::vector<int> motor_positions_; // Vector to store motor positions
  std::vector<bool> motor_directions_; // Vector to store motor directions (true = forward, false = inverse)
  int motor_count_; // Number of motors
  int min_position_; // Minimum position value
  int max_position_; // Maximum position value

  /*!
   * Move motor in the specified direction and check if it has reached the min or max position
   * @param motor_id The ID of the motor to move
   * @return true if the motor has reached either limit, false otherwise
  */
  bool moveMotor(int motor_id) {

    // std::cout << "id:" << motor_id << " pos/dir " << motor_positions_.at(motor_id) << " / " << motor_directions_.at(motor_id) << std::endl;

    // Simulate moving motor to min and max positions
    if (motor_directions_.at(motor_id)) {
      // Forward direction: increment position
      motor_positions_.at(motor_id) ++;
    } else {
      // Inverse direction: decrement position
      motor_positions_.at(motor_id) --;
    }

    // std::cout << "id:" << motor_id << " pos/dir " << motor_positions_.at(motor_id) << " / " << motor_directions_.at(motor_id) << std::endl;

    // Check if the motor has reached the min or max position 
    return checkOrientations(motor_id);
  }

  /*!
   * Check if the motor has reached the min or max position
   * @param motor_id The ID of the motor to check
   * @return true if the motor has reached either limit, false otherwise
  */
  bool checkOrientations(int motor_id) {

    bool reached_min = (motor_positions_.at(motor_id) == min_position_);
    bool reached_max = (motor_positions_.at(motor_id) == max_position_);

    if (reached_min) {
      motor_directions_.at(motor_id) = true; // Change direction to inverse
    }
    if (reached_max) {
      motor_directions_.at(motor_id) = false; // Change direction to inverse
    }

    // std::cout << "id:" << motor_id << " pos/dir " << motor_positions_.at(motor_id) << " / " << motor_directions_.at(motor_id) << std::endl;
    // std::cout << "Reached min: " << reached_min << " Reached max: " << reached_max << std::endl;

    return (reached_min or reached_max);

  }


  void doChecks(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // Prepare response
    response->success = true;
    response->message = "";
    RCLCPP_INFO(this->get_logger(), "Received request to check motors...");

    // Iterate over all motors to perform the check
    for (int i = 0; i < motor_count_; i++) {
      RCLCPP_INFO(this->get_logger(), "Checking motor %i", i);
      auto res = moveMotor(i);
      // If something fails, change response `success` to false and add info to the `message`
      if (res) {
        response->success = false;
        response->message += " Motor " + std::to_string(i) + " Limit. ";
      }
    }

    RCLCPP_INFO(this->get_logger(), "Sending back response...");
  }
};

int main(int argc, char **argv) {
  // Initiate the rclcpp library
  rclcpp::init(argc, argv);

  // Create an instance of the Motors class
  auto node = std::make_shared<Motors>();

  // Spin the node until it's terminated
  rclcpp::spin(node);
  
  rclcpp::shutdown();
}