using System;
using System.Text;
using System.Threading.Tasks;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using UnityEngine;

public class MQTTManager : MonoBehaviour
{
    private IMqttClient mqttClient;

    async void Start()
    {
        // Initialize MQTT client
        var factory = new MqttFactory();
        mqttClient = factory.CreateMqttClient();

        // MQTT options
        var options = new MqttClientOptionsBuilder()
            .WithClientId("UnityClient")
            .WithTcpServer("192.168.0.101", 1883) // Replace with your broker's IP and port
            .Build();

        // Connect to MQTT broker
        mqttClient.UseConnectedHandler(async e =>
        {
            Debug.Log("Connected to MQTT broker!");
            await mqttClient.SubscribeAsync(new MqttTopicFilterBuilder().WithTopic("test/topic").Build());
        });

        mqttClient.UseApplicationMessageReceivedHandler(e =>
        {
            string topic = e.ApplicationMessage.Topic;
            string payload = Encoding.UTF8.GetString(e.ApplicationMessage.Payload);
            Debug.Log($"Message received: {topic} -> {payload}");
        });

        await mqttClient.ConnectAsync(options);
    }

    public async Task PublishMessage(string topic, string message)
    {
        if (mqttClient.IsConnected)
        {
            var mqttMessage = new MqttApplicationMessageBuilder()
                .WithTopic(topic)
                .WithPayload(message)
                .Build();
            await mqttClient.PublishAsync(mqttMessage);
            Debug.Log($"Message published: {topic} -> {message}");
        }
    }

    private async void OnApplicationQuit()
    {
        if (mqttClient.IsConnected)
        {
            await mqttClient.DisconnectAsync();
        }
    }
}
