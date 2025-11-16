# RR UDP Server

UDP server using transport drivers to create UDP server.  This service will read UDP packets published to /udp_read topic, as
udp_msgs/msg/UdpPacket message. The will then be deserialized and pushed to the appropriate topics.

The primary role of this node is to create an interface with custom service, for the purposes of diagnosing the robot.

```bash

sudo apt-get install -y protobuf-compiler libprotobuf-dev
```

## DEBUG

```bash
 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_C_FLAGS_RELWITHDEBINFO="-g -O0" -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O0"  --packages-select rr_udp_server
 ros2 run --prefix 'gdbserver localhost:3000' rr_udp_server rr_udp_server_node -r /udp_read:=/udp_bridge_node_cmd/udp_read
```
