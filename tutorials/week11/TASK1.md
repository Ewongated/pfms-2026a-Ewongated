## TASK 1 - Client and Service Node

We will use the package `srv_client_checks` provided. 

This contains an example of using services in two way communication where we have both `service node` and  `client node`. This was based on [example from foxglove](https://foxglove.dev/blog/creating-ros2-services). In the example we have two nodes that need to communicate. Unlike topics, which follow a publish-subscribe model to send each other information, services follow a request-response model.

With services, a client node must first **call** the server in order to send a **request**. This means that in the server-client mode, nodes do not use a  communication stream until it’s absolutely needed, preventing  unnecessary bandwidth usage. Then, the client node must wait for a **response** from the server (or until a timeout is reached). For this reason, services can confirm communication between nodes.

![](https://cdn.prod.website-files.com/66a37d395dfadcdb65dcdf45/66e476da786ae6b52b1def20_hero.webp)

Examine the files, look at the narrative [from Foxglove dev's](https://foxglove.dev/blog/creating-ros2-services). To run code in two terminals run

```bash
ros2 run srv_client_checks diagnostics_node
```

And

```bash
ros2 run srv_client_checks motor_node
```

Open the files and answer following questions

- What is the name of the package?
- What dependencies does it have?
- How many executable(s) are there?
- What are the names of the nodes?
- How many topics is each node listening to?
- What is the server and what client node
- Look at syntax for client/server, what are they similar to?
- What does std:::bind do?
- What is the other different between the code for the two nodes?
- What is different between our supplied solution and that of foxglove or ROS webiste?

You will note that the `motor_node` can take parameters when running, which changes some settings. To pass the parameters you can execute the run with additional parameters. Looking at the code you will see `max_position` has a default value `5` while we can change it as below.

```bash
ros2 run srv_client_checks motor_node --ros-args -p max_position:=10
```