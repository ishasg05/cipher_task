#include "rclcpp/rclcpp.hpp"
#include "cipher_interfaces/msg/cipher_message.hpp"
#include "cipher_interfaces/srv/cipher_answer.hpp"
#include <string>
#include <iostream>
#include <memory>

static std::string encode(const std::string &input, int shift) {
    std::string ciphered = "";
    
    for (char c : input) {
        char init;
        if (isalpha(c)) {  
            if (isupper(c)) {
                init = 'A';
            } else {
                init = 'a';
            }
            char encoded_char = (c - init + shift) % 26 + init;
            ciphered += encoded_char;  
        } else {
            ciphered += c;
        }
    }
    return ciphered;
}

class CipherNode : public rclcpp::Node {
public:
    CipherNode() : Node("caesar_node") {
        publisher_ = this->create_publisher<cipher_interfaces::msg::CipherMessage>("cipher_message", 10);
        client_ = this->create_client<cipher_interfaces::srv::CipherAnswer>("cipher_answer");
    }

    void run() {
        std::string message;
        int key;
        std::cout << "Enter a message to encode: ";
        std::getline(std::cin, message);
        std::cout << "Enter the Caesar cipher key: ";
        std::cin >> key;

        std::string encoded_message = encode(message, key);

        auto cipher_msg = cipher_interfaces::msg::CipherMessage();
        cipher_msg.message = encoded_message;
        cipher_msg.key = key;

        RCLCPP_INFO(this->get_logger(), "publishing encoded message: '%s'", encoded_message.c_str());
        publisher_->publish(cipher_msg);

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "waiting for the service to become available...");
        }

        auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
        request->answer = message;

        auto future = client_->async_send_request(request);
        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->result) {
                RCLCPP_INFO(this->get_logger(), "yay, right encoding sent!");
            } else {
                RCLCPP_INFO(this->get_logger(), "wrong :(");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "service call failed.");
        }

        rclcpp::shutdown(); 
    }

private:
    rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher_;
    rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CipherNode>();
    node->run();  
    return 0;
}
