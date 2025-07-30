#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "collatz_core/msg/num.hpp"

#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 1234
#define TARGET_PORT 4321

class Collatz_Cpp_Node : public rclcpp::Node
{
public:
    Collatz_Cpp_Node()
    : Node("collatz_process_cpp")
    {   
        
        RCLCPP_INFO(this->get_logger(), "Cpp Node Started");

        //decalre the starting number as in input parameter
        this->declare_parameter("starting_number", 100000);
        
        this->declare_parameter("target_IP", "127.0.0.1");

        starting_number_ = get_parameter("starting_number").as_int();

        if(starting_number_ <= 0 || starting_number_ > 1000000) {
            RCLCPP_WARN(this->get_logger(), "Starting number must be positive and below 1000000");
            RCLCPP_INFO(this->get_logger(), "Setting starting number to 100000 as default");
            starting_number_ = 100000;
        }

        TARGET_IP_ = get_parameter("target_IP").as_string();

        RCLCPP_INFO(this->get_logger(), "Starting number: %d", starting_number_);


        //create a presistent socket to listen for incoming data
        server_sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sock_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Socket creation failed (receive)");
            rclcpp::shutdown();
            return;
        }

        //allow socker address reuse
        int opt = 1;
        setsockopt(server_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        //socket configuration
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET; //IPv4
        server_addr.sin_port = htons(PORT); //Listening port
        server_addr.sin_addr.s_addr = INADDR_ANY;
        
        //bind the socket to the address and port
        if (bind(server_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Bind failed (receive)");
            close(server_sock_);
            rclcpp::shutdown();
            return;
        }

        //listen for incoming connections
        if (listen(server_sock_, 5) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Listen failed (receive)");
            close(server_sock_);
            rclcpp::shutdown();
            return;
        }

        //publisher for plotting
        publisher_ = this->create_publisher<collatz_core::msg::Num>(
        "collatz_number", 10);
    }

    //main process of the collatz conjecture
    void collatz_process() {
        collatz_core::msg::Num data;
        data.num = starting_number_;

        //the loop to process odd numbers, send even and receive odd numbers
        //until we receive the number 1 from python and exit the loop
        while (data.num != 1) {

            if (data.num % 2 == 1) {
                data.num = data.num * 3 + 1;
            }

            //publish & sleep a short time for better logging and visualizing
            graph_publisher(data);
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            //send the data to python program
            send_data(data);
            
            //receive the data from python program
            data = receive_data();

            //publish & sleep a short time for better logging and visualizing
            graph_publisher(data);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        
        //end of process
        RCLCPP_INFO(this->get_logger(), "cpp: sequence reached 1, exiting...");
        rclcpp::shutdown();
    }


    void send_data(const collatz_core::msg::Num& data) {

        // serialize ROS message
        rclcpp::SerializedMessage serialized_data(1024);
        this->serializer.serialize_message(&data, &serialized_data);

        // create socket for sending data
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed (send)");
            return;
        }

        // set up target address
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(TARGET_PORT);
        inet_pton(AF_INET, TARGET_IP_.c_str(), &server_addr.sin_addr);

        // try to connect to python node
        int attempts = 0;
        while (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0 && attempts < 5) {
            RCLCPP_WARN(this->get_logger(), "Retrying connect... [%d]", attempts);
            attempts++;
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        // log error after 5 attempts
        if (attempts == 5) {
            RCLCPP_ERROR(this->get_logger(), "Connection to Python failed (send)");
            close(sock);
            return;
        }

        // send the serialized data
        ssize_t sent = send(sock, serialized_data.get_rcl_serialized_message().buffer,
                            serialized_data.size(), 0);
        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Send failed");
        } else {
            RCLCPP_INFO(this->get_logger(), "cpp: Sent: %ld", data.num);
        }

        //close the socket
        close(sock);    
    }
    
    collatz_core::msg::Num receive_data() {
        
        //a ros buffer to store serialized data
        rclcpp::SerializedMessage serialized_data(1024);

        //listen on the chosen port for incoming data
        int client_sock = accept(server_sock_, NULL, NULL);
        //log error if accept failed
        if (client_sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Accept failed (receive)");
            return collatz_core::msg::Num();
        }

        //receive the data 
        char buffer[1024];
        ssize_t received = recv(client_sock, buffer, sizeof(buffer), 0);
        close(client_sock);

        //log error if not received
        if (received <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Receive failed");
            return collatz_core::msg::Num();
        }

        //assisn the received bytes into the serialized variable before deserializing
        serialized_data.get_rcl_serialized_message().buffer_length = received;
        memcpy(serialized_data.get_rcl_serialized_message().buffer, buffer, received);

        //deserialize the receive bytes into a ros message and return
        collatz_core::msg::Num data;
        this->serializer.deserialize_message(&serialized_data, &data);
        RCLCPP_INFO(this->get_logger(), "cpp: Received: %ld", data.num);
        return data;
    }

    //starting flag to be recived from python
    collatz_core::msg::Num starting_flag_;

    //destructor to make sure the socket is closed after the program is done
    ~Collatz_Cpp_Node() {
        if (server_sock_ >= 0){
            close(server_sock_);
        }   
    }

private:
    
    //publish data on a topic to be plotted
    void graph_publisher(const collatz_core::msg::Num& data){
        publisher_->publish(data);
    }
    
    //class members
    rclcpp::Publisher<collatz_core::msg::Num>::SharedPtr publisher_;
    int starting_number_;
    int server_sock_;
    std::string TARGET_IP_;
    rclcpp::Serialization<collatz_core::msg::Num> serializer;//serializer object

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Collatz_Cpp_Node>();

    //wait for a data from python, check it and start the process if receive -1
    while (true) {
        RCLCPP_INFO(node->get_logger(), "Waiting for the python node get ready...");
        node->starting_flag_ = node->receive_data();

        if (node->starting_flag_.num == -1) {
            break;
        } else {
            RCLCPP_WARN(node->get_logger(), "Python node didn't send the right value, listening again...");
        }
    }
    RCLCPP_INFO(node->get_logger(), "Python node is ready, starting collatz process");
    
    //start the loop
    node->collatz_process();
    return 0;
}
