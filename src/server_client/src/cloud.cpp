// #define DEBUG_PING
// #define PING_TIMEOUT 3

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <json.hpp>

#include "messages/msg/data.hpp"
typedef messages::msg::Data Data;
#include "messages/srv/connection_request.hpp"
typedef messages::srv::ConnectionRequest ConnectionRequest;

using namespace std::chrono_literals;
using Json = nlohmann::json;
typedef long long unsigned int Id;

class Cloud : public rclcpp::Node
{
	rclcpp::Service<ConnectionRequest>::SharedPtr connection_request;
	rclcpp::Subscription<Data>::SharedPtr connection_from_client;
	rclcpp::TimerBase::SharedPtr pinger;

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
					RCLCPP_INFO(this->get_logger(), "Client disconnected with id '%ld'", req->close_id);
				} else if(req->close == false) {
					// Register new client
					res->client_id = this->get_next_id();
					RCLCPP_INFO(this->get_logger(), "Client connected with id '%ld'", res->client_id);
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
	std::function<void(const Data::SharedPtr)> process_client_data = [&](const Data::SharedPtr data) {
		RCLCPP_INFO(this->get_logger(), "Got data from client with id '%ld'", data->id);
	};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Cloud>());
	rclcpp::shutdown();
	return 0;
}
