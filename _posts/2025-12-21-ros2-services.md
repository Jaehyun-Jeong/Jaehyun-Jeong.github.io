---
title: Basic Guide to build and run ROS 2 Services (Python & C++)
tags: ros2
article_header:
  type: cover
pseudocode: true
---

If you don't know about ROS 2 Topics, go to [this](https://jaehyun-jeong.github.io/2025/12/09/ros2-topics.html) page and learn.

**Topics are used for data streams (unidirectional), and Services for a client/server interaction (bidirectional).** First, Services can work in synchronous or asynchronous manner. If the service is synchronous, the client send a request and block until receiving response. However, if the service is asynchronous, the client send a request and register callback function for the response and continue its execution. If the server responded, the callback function triggered. Second Services are defined by name and the pair of messages. One message is Request and other message is Response. Last but not least, a service can only exist once, but can have multiple clients.

## Simple Python code

### server

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server_ = self.create_service(
            AddTwoInts,
            "add_two_ints",  # Use a verb for service name
            self.callback_add_two_ints,
        )
        self.get_logger().info("Add Two Ints server has been started")

    def callback_add_two_ints(
        self,
        request: AddTwoInts.Request,
        response: AddTwoInts.Response
    ):
        response.sum = request.a + request.b
        self.get_logger().info(
            str(request.a) + " + " + str(request.b) + " = " + str(response.sum)
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Client

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")

    client = node.create_client(
        AddTwoInts,
        "add_two_ints"
    )
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Add Two Ints server...")

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    future = client.call_async(request)  # client.call (sync)
    # Spin until getting the response
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    node.get_logger().info(
        str(request.a) + " + " + str(request.b) + " = " + str(response.sum)
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
*Non-OOP method*

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Add Two Ints server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client_.call_async(request)
        # To add another argument, arguments must be wrapped with partial
        future.add_done_callback(partial(
            self.callback_call_add_two_ints, request=request
        ))

    def callback_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info(
            str(request.a) + " + " + str(request.b) + " = " + str(response.sum)
        )


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.call_add_two_ints(2, 7)
    node.call_add_two_ints(1, 4)
    node.call_add_two_ints(10, 20)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```
*OOP method*

**NOTE: In the OOP method, "rclpy.spin_until_future_complete(node, future)" is not required since the class is already spinning. Instead of this, it is required to add callback function with "future.add_done_callback".**

## Simple C++ code

### Server

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;


class AddTwoIntsServerNode : public rclcpp::Node{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2)
        );
        RCLCPP_INFO(this->get_logger(), "Add Two Ints Service has been started");
    }
private:
    void callbackAddTwoInts(
        const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
        const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Client

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop");

    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(1s)){
        RCLCPP_WARN(node->get_logger(), "Waiting for the server...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 6;
    request->b = 2;

    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, future);

    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);

    rclcpp::shutdown();
    return 0;
}
```
*Non-OOP method*

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;


class AddTwoIntsClientNode : public rclcpp::Node{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void callAddTwoInts(int a, int b){
        while(!this->client_->wait_for_service(1s)){
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        client_->async_send_request(
            request,
            std::bind(&AddTwoIntsClientNode::callbackCallAddInts, this, _1)
        );
    }

private:

    void callbackCallAddInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    node->callAddTwoInts(10, 5);
    node->callAddTwoInts(10, 15);
    node->callAddTwoInts(12, 7);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
```
*OOP method*

## ROS 2 commands for services

```bash
ros2 service -h

ros2 service list
# OUT
# example_interfaces/srv/AddTwoInts

# put the output into ros2 interface command
ros2 interface show example_interfaces/srv/AddTwoInts
# OUT
# int64 a
# int64 b
# ---
# int64 sum

# Then you can test this server like the below command
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 3}"
# OUT
#
# waiting for service to become available...
# requester: making request: example_interfaces.srv.AddTwoInts_Request(a=7, b=3)
#
# response:
# example_interfaces.srv.AddTwoInts_Response(sum=10)
```
