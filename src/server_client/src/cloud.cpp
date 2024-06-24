// Console everything that is happening to the node
//#define VERBOSE

// Console only the important state changes
//#define VERBOSE_IMPORTANT_ONLY

// Uncomment the following two lines to have constant default pings
//	happening towards the clients with action "0" and dummy data
//#define DEBUG_PING
//#define PING_TIMEOUT 3

// Includes
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <json.hpp>

// Ros2 message interfaces
#include "messages/msg/data.hpp"
typedef messages::msg::Data Data;
#include "messages/srv/connection_request.hpp"
typedef messages::srv::ConnectionRequest ConnectionRequest;

// Type definitions and usings
using namespace std::chrono_literals;
using Json = nlohmann::json;
typedef long long unsigned int Id;

// Cloud node
class Cloud : public rclcpp::Node
{
	rclcpp::Service<ConnectionRequest>::SharedPtr connection_request;
	rclcpp::Subscription<Data>::SharedPtr connection_from_client;
	#ifdef DEBUG_PING
	rclcpp::TimerBase::SharedPtr pinger;
	#endif

	Id last_id = 0;
	std::map<std::string, rclcpp::Publisher<Data>::SharedPtr> clients;

	inline Id get_next_id() {
		auto publisher_name = "connection_to_client"+(++last_id);
		clients.insert({
			publisher_name,
			this->create_publisher<Data>(publisher_name, 10)
		});
		return last_id;
	}

public:
	Cloud() : Node("cloud") {
		// Service
		this->connection_request = this->create_service<ConnectionRequest>("connection_request", [&](
			const ConnectionRequest::Request::SharedPtr req, const ConnectionRequest::Response::SharedPtr res) {
				if (req->close == true) {
					this->clients.erase("connection_to_client"+req->close_id);
					#if defined(VERBOSE) || defined(VERBOSE_IMPORTANT_ONLY)
					RCLCPP_INFO(this->get_logger(), "Client disconnected with id '%ld'", req->close_id);
					#endif
				} else if(req->close == false) {
					// Register new client
					res->client_id = this->get_next_id();
					#if defined(VERBOSE) || defined(VERBOSE_IMPORTANT_ONLY)
					RCLCPP_INFO(this->get_logger(), "Client connected with id '%ld'", res->client_id);
					#endif
				}
			}, 10);
		// Subscriber
		this->connection_from_client = this->create_subscription<Data>("connection_to_cloud", 10, this->process_client_data);
		#ifdef DEBUG_PING
		// Debug timer
		this->pinger = this->create_wall_timer(std::chrono::seconds(PING_TIMEOUT), [&]() {
			Data message = Data();
			message.id = 25565;
			message.action = 0;
			Json data = {
				{"name", "Priszi"},
				{"age", 22},
				{"height", 1.74},
				{"favourite_animal", {
					{"name", "cat"},
					{"carnivore", true},
					{"cute", true}
				}},
				{"drinks_coffe", false}
			};
			message.data = data.dump();
			for(auto client : this->clients) {
				client.second->publish(message);
			}
			RCLCPP_INFO(this->get_logger(), "Pinged all clients (%ld)", this->clients.size());
		});
		#endif
	}

private:
// Process data coming from the clients
	std::function<void(const Data::SharedPtr)> process_client_data = [&](const Data::SharedPtr msg) {
		#ifdef VERBOSE
		RCLCPP_INFO(this->get_logger(), "Got data from client with id '%ld'", msg->id);
		#endif
		// Parse the string data from the recieved message
		Json data = Json::parse(msg->data);
		// Define some action when recieved a message with a given action
		switch(msg->action) {
			case 1:
				// Do something with data if action is "1"
				this->process_action_1_data(data);
				break;
			default:
				#if defined(VERBOSE) || defined(VERBOSE_IMPORTANT_ONLY)
				RCLCPP_INFO(this->get_logger(), "Got data with unknown action id (%ld)", msg->action);
				#endif
				break;
		}
	};

	// Here comes the function declarations of the actions' callbacks
	void process_action_1_data(Json&);
};

// Node spin (aka. start) logic
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Cloud>());
	rclcpp::shutdown();
	return 0;
}

// Action callback definitions (could be put in different file and included that in this one for cleaner code)
void Cloud::process_action_1_data(Json& data) {
	auto some_data = std::string(data["name"]).c_str();
	RCLCPP_INFO(this->get_logger(), "I got action 1 data: %s.", some_data);
}
