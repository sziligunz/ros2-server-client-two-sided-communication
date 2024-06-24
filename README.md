# ROS2-SERVER-CLIENT-TWO-SIDED-COMMUNICATION

## About

This repository contains the implementation of a communication system between **1 server** (named "cloud" in the implementations) and **N clients**. It is written in **C++** using **ROS2 iron**.

The messages between server and clients are sent via publishers and subscribers and represented are in the messages with JSON strings, which can be dumped and parsed with the included [JSON](https://github.com/nlohmann/json) library created by *NLohmann*.

This system only works if the **nodes are located in the same local network** and the environment variable **ROS_DOMAIN_ID is set** to the same number. This ensures that the nodes can communicated with each other when located on the same local network. More about what is a [ROS_DOMAIN_ID](https://docs.ros.org/en/iron/Concepts/Intermediate/About-Domain-ID.html).

When trying to communicate on seperate local networks a VPN can be used to achive the same effect, but I haven't tested this myself.

## When is this useful?

This solution is **ideal for high performance local network communication**. Since it's written in C++ using ROS2 it makes this a fast and small, multiplatform alternative to a traditional server.

Hower i **don't recommend** using this as a **non-local network solution**, because a VPN is necessary to be set up, which slows down the communication.

## Build and run

**Linux:**

1. When standing in the repository's root folder run the following command to build the messages used by the nodes:

    ```sh
    colcon build --packages-select messages
    ```

2. Than we have to source the newly built messages:

    ```sh
    source install/setup.sh
    ```

3. Now we can build the nodes:

    ```sh
    colcon build --packages-select server_client
    ```

    *We don't have to source the install again.*

4. The server can be run with the following command:

    ```sh
    ros2 run server_client cloud
    ```

5. A client can be run with the following command:

    ```sh
    ros2 run server_client client
    ```

## Shutdown

Shuting down a client is **not** as simple as pressing **`CTRL+C`** since we have to notify the server (cloud) that a client will be disconnecting. This is why we have to **run the following command before** `CTRL+C`:

```sh
ros2 lifecycle set client shutdown
```

The command above ensures that a request will be sent to the server (cloud) that contains the closing clients ID.

## Usage

### Defining custom data processing logic

The following code shows how to implement the processing of data with a given action (can be found in file `client.cpp` from row 74):

```c++
// Process data coming from the cloud
std::function<void(const Data::SharedPtr)> process_cloud_request = [&](const Data::SharedPtr msg) -> void {
    // Parse the string data from the recieved message
    Json data = Json::parse(msg->data);
    // Define some action when recieved a message with a given action
    switch(msg->action) {
        ////////////////////////////////////////////
        // ADD YOUR OWN CASES WITH YOUR OWN LOGIC //
        ////////////////////////////////////////////
        case 0:
            // Do something with data if action is "0"
            this->process_action_0_data(data);
            break;
        default:
            break;
    }
};
```

The following code demonstates how to write custom data processing logic (can be found in file `client.cpp` from row 139). *Notice that this function is supposed to work only with data having action id "0".*

```c++
// Action callback definitions (could be put in different file and included that in this one for cleaner code)
void Client::process_action_0_data(Json& data) {
    auto some_data = std::string(data["name"]).c_str();
    RCLCPP_INFO(this->get_logger(), "I got action 0 data: %s.", some_data);
}
```

### `VERBOSE` and `VERBOSE_IMPORTANT_ONLY` modes

Both cloud and client have the ability to log what is happening to them. Either modes can be enabled by uncommening the appropriate lines located at the beggining of both `cloud.cpp` and `client.cpp`. `VERBOSE` mode logs out everything that it can and `VERBOSE_IMPORTANT_ONLY` logs out only the most important state changes.
