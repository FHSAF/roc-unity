FROM ros:melodic-ros-base

# Add ROS GPG key and update package lists
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 \
    && apt-get update

# Install required ROS packages and system utilities
RUN apt-get install -y \
    vim \
    iputils-ping \
    net-tools \
    python-pip \
    ros-melodic-robot-state-publisher \
    ros-melodic-moveit \
    ros-melodic-rosbridge-suite \
    ros-melodic-joy \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-tf2-web-republisher \
    dos2unix

# Install Python dependencies
RUN pip install --no-cache-dir \
    rospkg \
    jsonpickle \
    paho-mqtt

# Set ROS workspace environment variable
ENV ROS_WORKSPACE=/catkin_ws

# Copy ROS workspace source directories
COPY ./ROS/src/moveit_msgs $ROS_WORKSPACE/src/moveit_msgs
COPY ./ROS/src/niryo_moveit $ROS_WORKSPACE/src/niryo_moveit
COPY ./ROS/src/niryo_one_ros $ROS_WORKSPACE/src/niryo_one_ros
COPY ./ROS/src/niryo_one_urdf $ROS_WORKSPACE/src/niryo_one_urdf
COPY ./ROS/src/ros_tcp_endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint

COPY ./ROS/src/mqtt_bridge_package /catkin_ws/src/mqtt_bridge_package


# Copy setup scripts
COPY ./docker/set-up-workspace /setup.sh
COPY docker/tutorial /

# Convert files to UNIX format and make scripts executable
RUN /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix" \
    && dos2unix /tutorial \
    && dos2unix /setup.sh \
    && chmod +x /setup.sh \
    && /setup.sh \
    && rm /setup.sh

# Make ROS Python scripts executable
RUN chmod +x /catkin_ws/src/niryo_moveit/scripts/*.py
RUN chmod +x /catkin_ws/src/mqtt_bridge_package/scripts/*.py

# Expose ports if required (e.g., for debugging or other services)
EXPOSE 11311 1883

# Set the working directory
WORKDIR $ROS_WORKSPACE

# Set entrypoint
ENTRYPOINT ["/tutorial"]
