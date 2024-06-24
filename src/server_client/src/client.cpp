// Console everything that is happening to the node
//#define VERBOSE

// Console only the important state changes
//#define VERBOSE_IMPORTANT_ONLY

// Includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <json.hpp>

// Ros2 message interfaces
#include "messages/msg/data.hpp"
typedef messages::msg::Data Data;
#include "messages/srv/connection_request.hpp"
typedef messages::srv::ConnectionRequest ConnectionRequest;

// Type definitions and usings
using namespace std::chrono_literals;
using Json = nlohmann::json;

// Client node
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
			#if defined(VERBOSE) | defined(VERBOSE_IMPORTANT_ONLY)
			RCLCPP_INFO(this->get_logger(), "Couldn't connect to cloud");
			#endif
			return;
		}
		this->connection_request->async_send_request(
			std::make_shared<ConnectionRequest::Request>(), [&](const rclcpp::Client<ConnectionRequest>::SharedFuture res) {
				#if defined(VERBOSE) | defined(VERBOSE_IMPORTANT_ONLY)
				RCLCPP_INFO(this->get_logger(), "Connected to cloud");
				#endif
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
		#ifdef VERBOSE
		RCLCPP_INFO(
			this->get_logger(),
			"Got data from server:\n\tid: '%ld'\n\taction: '%d'\n\tmessage: '%s'",
			msg->id, msg->action, msg->data.c_str()
		);
		#endif
		// Parse the string data from the recieved message
		Json data = Json::parse(msg->data);
		// Define some action when recieved a message with a given action
		switch(msg->action) {
			case 0:
				// Do something with data if action is "0"
				this->process_action_0_data(data);
				break;
			default:
				#if defined(VERBOSE) || defined(VERBOSE_IMPORTANT_ONLY)
				RCLCPP_INFO(this->get_logger(), "Got data with unknown action id (%ld)", msg->action);
				#endif
				break;
		}
	};

	// Lifecycle callback definitions
	typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;
	typedef rclcpp_lifecycle::State State;
	CallbackReturn on_shutdown(const State&) {
		#ifdef VERBOSE
		RCLCPP_INFO(this->get_logger(), "Node is shuting down");
		#endif
		// If we connected to the cloud send disconnect request
		if(this->connection_to_cloud != nullptr) {
			if (!this->connection_request->wait_for_service(5s)) {
				#if defined(VERBOSE) | defined(VERBOSE_IMPORTANT_ONLY)
				RCLCPP_INFO(this->get_logger(), "Couldn't connect to cloud");
				#endif
				return CallbackReturn::FAILURE;
			}
			auto req = std::make_shared<ConnectionRequest::Request>();
			req->close = true;
			req->close_id = this->client_id;
			this->connection_request->async_send_request(req);
			#if defined(VERBOSE) | defined(VERBOSE_IMPORTANT_ONLY)
			RCLCPP_INFO(this->get_logger(), "Sent disconnect request to cloud");
			#endif
		} else {
			#ifdef VERBOSE
			RCLCPP_INFO(this->get_logger(), "Disconnection request is unnesceserry since this node haven't connected to the cloud");
			#endif
		}
		return CallbackReturn::SUCCESS;
	};
	CallbackReturn on_configure(const State&) {return CallbackReturn::SUCCESS;}
	CallbackReturn on_activate(const State& state) {LifecycleNode::on_activate(state); return CallbackReturn::SUCCESS;}
	CallbackReturn on_deactivate(const State& state) {LifecycleNode::on_deactivate(state); return CallbackReturn::SUCCESS;}
	CallbackReturn on_cleanup(const State&) {return CallbackReturn::SUCCESS;}

	// Here comes the function declarations of the actions' callbacks
	void process_action_0_data(Json&);
};

// Node spin (aka. start) logic
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

// Action callback definitions (could be put in different file and included that in this one for cleaner code)
void Client::process_action_0_data(Json& data) {
	auto some_data = std::string(data["name"]).c_str();
	RCLCPP_INFO(this->get_logger(), "I got action 0 data: %s.", some_data);
}
