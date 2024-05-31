//#pragma once

#include <string>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

/*

#include <stdio.h>
#include <string.h>
#include <MQTTClient.h>

// MQTT settings
#define ID "1"
#define BROKER "192.168.179.18"  // Dirección del broker MQTT
#define CLIENT_NAME "pico_"+ID
#define TOPIC "80"  // Tópico al que te suscribirás

*/

// MQTT settings
const String ID = "1";
const String BROKER = "192.168.1.104";
const String CLIENT_NAME = "pico_"+ID;
const String TOPIC = "80";

//const char* in_txt[256];
String in_txt;
bool callback = false;

WiFiClient espClient;
PubSubClient client(espClient); // Setup MQTT client

// ---- MQTT


// Handle incomming messages from the broker
void clientCallback(char* topic, byte* payload, unsigned int length) {
  String response;

  for (int i = 0; i < length; i++) {
    response += (char)payload[i];
  }
  // Serial.print("Message arrived [");
  // Serial.print(TOPIC.c_str());
  // Serial.print("] ");
  // Serial.println(response);
  in_txt = response;
  callback = true;

  // Obtén la longitud de la cadena original
  size_t len = response.length();

  // Reserva memoria para una cadena de caracteres (char[]) con el tamaño adecuado
  char *response_char = (char *)malloc(len + 1);  // +1 para el carácter nulo '\0'

  // Copia la cadena original en la cadena convertida
  response.toCharArray(response_char, len + 1);

  //return response;
}

void reconnectMQTTClient() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(CLIENT_NAME.c_str())) {
      Serial.print("connected to Broker: ");
      Serial.println(BROKER.c_str());
      // Topic(s) subscription
      client.subscribe(TOPIC.c_str());
    }
    else {
      Serial.print("Retying in 5 seconds - failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void createMQTTClient() {
  client.setServer(BROKER.c_str(), 1883);
  client.setCallback(clientCallback);
  reconnectMQTTClient();
}

/*

MQTTClient client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

void messageArrived(void* context, char* topicName, int topicLen, MQTTClient_message* message) {
    //printf("Mensaje recibido en el tópico: %s\n", topicName);
    //printf("Contenido del mensaje: %.*s\n", message->payloadlen, (char*)message->payload);

    // Supongamos que el mensaje es un texto simple y queremos guardarlo en una variable
    char receivedMessage[256];  // Se supone que el mensaje no excede los 256 caracteres
    snprintf(receivedMessage, sizeof(receivedMessage), "%.*s", message->payloadlen, (char*)message->payload);
    //printf("Mensaje recibido guardado en la variable: %s\n", receivedMessage);

    strncpy(in_txt, message, sizeof(in_txt)); // Copia el mensaje recibido en la variable in_txt
    callback = true;

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
}

void connectMQTTBroker() {
    MQTTClient_create(&client, BROKER, CLIENT_NAME, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    MQTTClient_setCallbacks(client, NULL, NULL, messageArrived, NULL);

    if (MQTTClient_connect(client, &conn_opts) == MQTTCLIENT_SUCCESS) {
            printf("connected to Broker: %s\n", BROKER);
            MQTTClient_subscribe(client, TOPIC, 0);
            return;
    } else {
            printf("Retying in 5 seconds - failed\n");
            sleep(5);  // Espera 5 segundos antes de intentar nuevamente
    }
}
*/