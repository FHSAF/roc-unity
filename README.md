
docker run -it --rm -p 10000:10000 unity-robotics:pick-and-place /bin/bash

roslaunch mqtt_bridge_package mqtt_node.launch


docker exec -it 0d9a6262dabe /bin/bash

rostopic pub /ros_to_mqtt std_msgs/String "Hello MQTT"

rostopic echo /ros_topic

source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
