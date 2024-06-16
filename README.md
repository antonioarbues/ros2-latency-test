# ros2-latency-test
Measure latency sending images from one remote host to another via ROS2 (you need to be on the same LAN).

## Example setup
Local
~~~
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_IP=<this_machine_ip>
export ROS_HOSTNAME=<this_machine_ip>
ros2 run speaker speaker
~~~

Remote
~~~
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_IP=<this_machine_ip>
export ROS_HOSTNAME=<this_machine_ip>
ros2 run listener listener
~~~
