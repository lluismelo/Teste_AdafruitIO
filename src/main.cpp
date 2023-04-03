#include <Arduino.h>
#include <ESP8266WiFi.h>

//#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>

const int saida = 2;

#define DHTPIN 5  
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define ssid "nome_rede"
#define pass "senha_rede"

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME  "name"
#define AIO_KEY       "************************"


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);


Adafruit_MQTT_Subscribe LED = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/monitoramento-testemqtt.led");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/monitoramento-testemqtt.temperatura");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/monitoramento-testemqtt.umidade");

void MQTT_connect();

void setup() {
  // put your setup code here, to run once:

  dht.begin();

  pinMode(saida, OUTPUT);
  
  Serial.begin(115200);
  delay(10);

  WiFi.begin(ssid,pass);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
  }
  
  mqtt.subscribe(&LED);
  //mqtt.subscribe(&Temperatura);
  //mqtt.subscribe(&Umidade);
}

uint32_t x=0;

void loop() {
  // put your main code here, to run repeatedly:

  MQTT_connect();
  int umid = (int)dht.readHumidity();
  int tempe = (int)dht.readTemperature();

  if(! temperature.publish(tempe))
      Serial.println(F("Falha na publicação da temperatura"));
  else
      Serial.println(F("Temperatura publicada"));
  if(! humidity.publish(umid))
      Serial.println(F("Falha na publicação da umidade"));
  else
      Serial.println(F("Umidade publicada"));


  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))){

    if (subscription == &LED){
        if(strcmp((char *)LED.lastread, "ON") == 0){
            digitalWrite(saida,LOW);
        }
        if(strcmp((char *)LED.lastread, "OFF") == 0){
            digitalWrite(saida,HIGH);
        }        
    }
  }

  if(! mqtt.ping()){
    mqtt.disconnect();}
}

void MQTT_connect(){
  int8_t ret;

  if(mqtt.connected()){
    return;}

  Serial.print("Conectando...");

  uint8_t retries = 3;
  while((ret = mqtt.connect()) != 0){
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reconectando em 5 segundos");
    mqtt.disconnect();
    delay(500);
    retries--;
    if(retries == 0){
      while(1);
    }
  }
  Serial.println("Conectado.");
  }