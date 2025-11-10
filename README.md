# RR UDP Server

UDP server using transport drivers to create UDP server.  This service will read UDP packets published to /udp_read topic, as
udp_msgs/msg/UdpPacket message. The will then be deserialized and pushed to the appropriate topics.

The primary role of this node is to create an interface with custom service, for the purposes of diagnosing the robot.


```bash

sudo apt-get install -y protobuf-compiler libprotobuf-dev
```
