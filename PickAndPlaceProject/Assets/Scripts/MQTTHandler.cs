using System;
using UnityEngine;
using UnityEngine.UI;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;

public class MQTTHandler : MonoBehaviour
{
    private IMqttClient mqttClient;
    private IMqttClientOptions mqttOptions;

    // Button references
    public Button publishButton;
    public Button subscribeButton;

    // Event for notifying other scripts of received MQTT messages
    public static event Action<string> OnMessageReceived;

    void Start()
    {
        // Initialize MQTT client
        var factory = new MqttFactory();
        mqttClient = factory.CreateMqttClient();

        mqttOptions = new MqttClientOptionsBuilder()
            .WithClientId("UnityClient")
            .WithTcpServer("192.168.0.101", 1883) // Replace with your broker's IP and port
            .Build();

        ConnectToBroker();

        // Attach button click events
        publishButton.onClick.AddListener(PublishMessage);
        subscribeButton.onClick.AddListener(SubscribeToTopic);
    }

    private async void ConnectToBroker()
    {
        try
        {
            await mqttClient.ConnectAsync(mqttOptions);
            Debug.Log("Connected to MQTT broker.");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to connect to broker: {ex.Message}");
        }
    }

    private async void PublishMessage()
    {
        if (mqttClient.IsConnected)
        {
            var message = new MqttApplicationMessageBuilder()
                .WithTopic("mqtt/unity_test")
                .WithPayload("Hello from Unity!")
                .WithExactlyOnceQoS()
                .Build();

            await mqttClient.PublishAsync(message);
            Debug.Log("Message published!");
        }
        else
        {
            Debug.LogError("MQTT client is not connected.");
        }
    }

    private async void SubscribeToTopic()
    {
        if (mqttClient.IsConnected)
        {
            await mqttClient.SubscribeAsync(new MqttTopicFilterBuilder()
                .WithTopic("mqtt/unity_test") // Replace with your desired topic
                .Build());

            mqttClient.UseApplicationMessageReceivedHandler(e =>
            {
                var payload = System.Text.Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
                Debug.Log($"Received message: {payload}");

                // Notify other scripts about the received message
                OnMessageReceived?.Invoke(payload);
            });

            Debug.Log("Subscribed to topic!");
        }
        else
        {
            Debug.LogError("MQTT client is not connected.");
        }
    }

    private async void OnDestroy()
    {
        if (mqttClient.IsConnected)
        {
            await mqttClient.DisconnectAsync();
        }
    }
}
