#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include <string>
#include <iostream>
#include <memory>

static std::string decode(const std::string &input, int shift) {
    std::string msg = "";
    
    for (char c : input) {
        char init;
        if (isalpha(c)) {  
            if (isupper(c)) {
                init = 'A';
            } else {
                init = 'a';
            }
            char decoded_char = (c - init - shift + 26) % 26 + init;
            msg += decoded_char; 
        } else {
            msg += c;
        }
    }
    return msg;
}

class CheckAnswerNode : public rclcpp::Node {
public:
    CheckAnswerNode() : Node("check_answer_node") {
        subscription_ = this->create_subscription<cipher_interfaces::msg::CipherMessage>(
            "cipher_message", 10, std::bind(&CheckAnswerNode::topic_callback, this, std::placeholders::_1));

        service_ = this->create_service<cipher_interfaces::srv::CipherAnswer>(
            "cipher_answer", std::bind(&CheckAnswerNode::validate_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription_;
    rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service_;

    std::string decoded_;

    void topic_callback(const cipher_interfaces::msg::CipherMessage::SharedPtr msg) {
        decoded_= decode(msg->message, msg->key);
        RCLCPP_INFO(this->get_logger(), "received encoded: '%s'",
                    msg->message.c_str());
    }

    void validate_callback(
        const std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> request,
        std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response> response) 
    {
        if (request->answer == decoded_) {
            response->result = true;
            RCLCPP_INFO(this->get_logger(), "decoded message '%s' was what we wanted, hooray", request->answer.c_str());
        } else {
            response->result = false;
            RCLCPP_WARN(this->get_logger(), "wrong :(. expected '%s', got '%s'.",
                        request->answer.c_str(), decoded_.c_str());
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CheckAnswerNode>());
    rclcpp::shutdown();
    return 0;
}
