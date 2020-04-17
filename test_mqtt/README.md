# test_mqtt

test_mqtt provides some nodes, messages and launch files to test the communication between ROS and an MQTT server


## Dependencies

`test_mqtt` uses `mqtt_bridge`. The standard version of mqtt_bridge can be installed using:

```
$ sudo apt-get install ros-kinetic-mqtt-bridge
```

But, this version doesn't use Quality of Service Level 2.
If you want to use qos level 2 you can use a modified version of mqtt_bridge, which
source code you can get place inside a catkin_ws/src folder by cloning the following
github repository:

git clone https://github.com/alejandrobrazabarba/mqtt_bridge

Furthermore, some python specific packages that should be installed using pip

```
$ python -m pip install paho.mqtt
$ python -m pip install inject
```

Optionally, we can install MQTT command line tools
```
$ sudo apt-get install mosquitto-clients
```
 Which will allow us to test the translation from ROS to MQTT and the other way around.


`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are serialized by json (or messagepack) for MQTT, and messages from MQTT are deserialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)

This limitation can be overcome by defining custom bridge class, though.


## Test

To test the correct creation of a geofence the launch file `test_cylindrical_geofence_creation.launch` includes all the necessary nodes to send geofence creation messages to STWS server on IP 95.216.180.100 and port 1883.

The message is generated in a ROS node which publishes it as a test_mqtt/GeofenceCreation msg that is received by mqtt_bridge_node.py through the rostopic `/geofence_creation`. Then, the message is translated into JSON format and sent to the server through MQTT topic `geofence_creation`


ROS to MQTT message translation can be checked by using mosquitto_sub

```
$ mosquitto_sub -h -c  95.216.180.100 -p 1883 -t geofence_creation -id 1234
```
If you want to test if the qos level is being used, you must use the -c option combined with the use of a client id.
If properly configured, the qos 2 will ensure that even when the subscriber disconnects temporarily (due to network connection issues maybe), it will receive the messages that were published while it was disconnected.

