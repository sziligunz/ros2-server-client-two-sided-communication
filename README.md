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

The following code contains the definition on how to process JSON data (can be found in file `client.cpp` from row 50):

```c++
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
```
