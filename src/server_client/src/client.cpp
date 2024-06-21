#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <json.hpp>

#include "messages/msg/data.hpp"
typedef messages::msg::Data Data;
#include "messages/srv/connection_request.hpp"
typedef messages::srv::ConnectionRequest ConnectionRequest;

using namespace std::chrono_literals;
using Json = nlohmann::json;

class Client : public rclcpp_lifecycle::LifecycleNode
{
	rclcpp::Client<ConnectionRequest>::SharedPtr connection_request;
    rclcpp::Publisher<Data>::SharedPtr connection_to_cloud;
	rclcpp::Subscription<Data>::SharedPtr connection_from_cloud;

	typedef long long unsigned int Id;
	Id client_id = 0;

public:
    Client() : LifecycleNode("client") {
		// Create client to request connection from cloud
		this->connection_request = this->create_client<ConnectionRequest>("connection_request", 10);
		// Request connection from cloud
		if (!this->connection_request->wait_for_service(5s)) {
			RCLCPP_INFO(this->get_logger(), "Couldn't connect to cloud");
			return;
		}
		this->connection_request->async_send_request(
			std::make_shared<ConnectionRequest::Request>(), [&](const rclcpp::Client<ConnectionRequest>::SharedFuture res) {
				RCLCPP_INFO(this->get_logger(), "Connected to cloud");
				// Save our client id recieved from cloud
				this->client_id = res.get()->client_id;
				// Create publisher towards cloud
        		this->connection_to_cloud = this->create_publisher<Data>("connection_to_cloud", 10);
				// Create subscriber from cloud
				this->connection_from_cloud = this->create_subscription<Data>("connection_to_client"+res.get()->client_id, 10, this->process_cloud_request);
			}
		);
    }

private:
	// Process data coming from the cloud
	std::function<void(const Data::SharedPtr)> process_cloud_request = [&](const Data::SharedPtr msg) -> void {
		RCLCPP_INFO(
			this->get_logger(),
			"Got data from server:\n\tid: '%ld'\n\taction: '%d'\n\tmessage: '%s'",
			msg->id, msg->action, msg->data.c_str()
		);
		// Parse the string data from the recieved message
		Json data = Json::parse(msg->data);
		// Define some logic when recieved a message with a given action
		switch(msg->action) {
			case 0:
				// Do something with the data
				data["name"] = "Kevin";
				data["favourite_animal"]["name"] = "dog";
				data["favourite_animal"]["cute"] = false;
				break;
			default:
				break;
		}
	};

	// Lifecycle callback definitions
	typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;
	typedef rclcpp_lifecycle::State State;
	CallbackReturn on_shutdown(const State&) {
		RCLCPP_INFO(this->get_logger(), "Node is shuting down");
		// If we connected to the cloud send disconnect request
		if(this->connection_to_cloud != nullptr) {
			if (!this->connection_request->wait_for_service(5s)) {
				RCLCPP_INFO(this->get_logger(), "Couldn't connect to cloud");
				return CallbackReturn::FAILURE;
			}
			auto req = std::make_shared<ConnectionRequest::Request>();
			req->close = true;
			req->close_id = this->client_id;
			this->connection_request->async_send_request(req);
			RCLCPP_INFO(this->get_logger(), "Sent disconnect request to cloud");
		} else {
			RCLCPP_INFO(this->get_logger(), "Disconnection request is unnesceserry since this node haven't connected to the cloud");
		}
		return CallbackReturn::SUCCESS;
	};
	CallbackReturn on_configure(const State&) {return CallbackReturn::SUCCESS;}
	CallbackReturn on_activate(const State& state) {LifecycleNode::on_activate(state); return CallbackReturn::SUCCESS;}
	CallbackReturn on_deactivate(const State& state) {LifecycleNode::on_deactivate(state); return CallbackReturn::SUCCESS;}
	CallbackReturn on_cleanup(const State&) {return CallbackReturn::SUCCESS;}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	// Some magic to be able to spin a class having type of LifecycleNode instead of Node
	rclcpp::executors::SingleThreadedExecutor executor;
	std::shared_ptr<Client> node = std::make_shared<Client>();
	executor.add_node(node->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
