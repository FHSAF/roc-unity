#!/usr/bin/env python

import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String

# ROS to MQTT callback
def ros_to_mqtt_callback(msg):
    mqtt_client.publish("mqtt/topic", msg.data)
    rospy.loginfo("Published to MQTT: %s", msg.data)

# MQTT to ROS callback
def on_mqtt_message(client, userdata, msg):
    ros_msg = String()
    ros_msg.data = msg.payload.decode()
    ros_pub.publish(ros_msg)
    rospy.loginfo("Received from MQTT: %s", ros_msg.data)

def main():
    global mqtt_client, ros_pub

    # Initialize ROS node
    rospy.init_node("mqtt_node")

    # ROS Publisher
    ros_pub = rospy.Publisher("ros_topic", String, queue_size=10)

    # ROS Subscriber
    rospy.Subscriber("ros_to_mqtt", String, ros_to_mqtt_callback)

    # MQTT Client Setup
    broker_address = "host.docker.internal"  # Use this for Docker Desktop
    # broker_address = "192.168.x.x"         # Use host IP for Linux
    mqtt_client = mqtt.Client()
    mqtt_client.on_message = on_mqtt_message
    mqtt_client.connect(broker_address, 1883, 60)
    mqtt_client.subscribe("mqtt/topic")

    # Start MQTT loop in a separate thread
    mqtt_client.loop_start()

    # Keep ROS spinning
    rospy.spin()

if __name__ == "__main__":
    main()
