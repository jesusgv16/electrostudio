#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"
#define DHTTYPE DHT11   // DHT 11

#define Relay1            16 //D0
#define Relay2            5//D1
#define Relay3            4 //D2
#define temp              0 //D3
#define PIR1              12 //D4
#define alarm_indicator   14 //D5   13 15
#define PIR2              2 //D6
    int last_temp=0;
    int auto_state;
    int Light2_State=1;
    int Fabanico=0;
    int Fabanico_last=2;
    int armed=0;
    int counter=7;
    int value_last=2;
    unsigned long previousMillisfoco=0;
    unsigned long timeon=1000;

#define WLAN_SSID       ""
#define WLAN_PASS       "e"

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""
DHT dht(temp, DHTTYPE);

    
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/


Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay1"); // FeedName
Adafruit_MQTT_Subscribe Light2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Relay2");
Adafruit_MQTT_Subscribe Alarma_status = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Alarm");
Adafruit_MQTT_Subscribe automatic = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/AutoLights");


Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish motion      = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/PIR1");
Adafruit_MQTT_Publish Abanico     = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Relay1");
Adafruit_MQTT_Publish foco        = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Relay2");


void MQTT_connect();

void setup() {

  dht.begin();
  Serial.begin(115200);

  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(temp,   INPUT);
  pinMode(PIR1,   INPUT);
  pinMode(PIR2,   INPUT);
  pinMode(alarm_indicator, OUTPUT);


  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Light1);
  mqtt.subscribe(&Light2);
  mqtt.subscribe(&Alarma_status);
  mqtt.subscribe(&automatic);

 
}

void loop() {

  MQTT_connect();

  
/************************* LECTURA DE SENSORES *********************************/
int temperature_data = (int)dht.readTemperature();
int motion_data= digitalRead(PIR1);
int value= digitalRead(PIR2);
unsigned long currentMillis = millis(); // grab current time


/******** PUBLICACION DE TEMPERATURA Y ENCENDIDO DE ABANICO***************/
if(temperature_data!=last_temp)
{
if (! temperature.publish(temperature_data))
        Serial.println(F("Failed to publish temperature"));
      else
      {
        Serial.print(F("\nSending Temperature value: "));
        Serial.print(String(temperature_data).c_str());
        Serial.print("...\n");
        delay(2000);
      } 

last_temp=temperature_data;
}      
                if(temperature_data>=30)
                   {
                 Fabanico=1;
                   if (Fabanico!=Fabanico_last)
                  {
                      Abanico.publish("0");
                      Serial.println("Encendiendo Abanico (Accion Automatica)\n");
                      Fabanico_last = Fabanico;
                  }
                   }
                 if (temperature_data<  30)
                 {
                   Fabanico=0;
                     if (Fabanico!=Fabanico_last)
                      {
                         Abanico.publish("1");
                        Serial.println("Apagando Abanico (Accion Automatica)\n");
                        Fabanico_last = Fabanico;
                       }
                 }
 

/******** SISTEMA DE SEGURIDAD***************/
           if (motion_data == HIGH && armed == HIGH  )
              {
                motion.publish(motion_data);
                Serial.println("MOTION DETECTED!!!!\t");
                Serial.println("NOT ALLOWED ACCESS!!!!...\t");
                Serial.println("Use Web interface to turn off alarm\n");
                armed=2;
              }
                  if (armed==1)
                 {
                  Serial.println("monitoring system enable use web interface to turn it off \n");
                  blinker();
                 
                 }       
                    if (armed==2)
                 {
                  Serial.println("MOTION DETECTED!!!!\t Use Web interface to turn off alarm\n");
                   digitalWrite(alarm_indicator, HIGH); 
                   Noise();
                 }     
/**********************************************************************/
/******** ENCENDIDO DE FOCOS POR PIR***************/


                    if (value == HIGH && auto_state==HIGH)
                      {
                        foco.publish(!value);
                        Serial.print("\t Turning Lights On  ");
                        value_last=value;

                        counter++;
                        counter++;
                        counter++;
                        Serial.print(counter); 
                        Serial.println(); 
                       }

                      if (value == LOW && auto_state==HIGH)
                      {
                        counter--;
                        Serial.print("\t still waiting!   ");
                        Serial.print(counter);
                        Serial.println();                    
                        }
                       if (counter<=0 && Light2_State==0)
                       {
                        Serial.println("empty room!!");
                        foco.publish("1");
                        Serial.println(counter); 
                       }
                       if (counter<=0 || counter==0)
                            {
                              counter=0;                                                
                            }
                                     
/******** PUBLICACIONES***************/
Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(3000))) 
  { 
    if (subscription == &Light1) {
      Serial.print(F("Abanico Got: "));
      Serial.println((char *)Light1.lastread);
      int Light1_State = atoi((char *)Light1.lastread);
      digitalWrite(Relay1, Light1_State);

    }
    if (subscription == &Light2) {
      Serial.print(F("Light2 Got: "));
      Serial.println((char *)Light2.lastread);
      Light2_State = atoi((char *)Light2.lastread);
      digitalWrite(Relay2, Light2_State);
    }

    if (subscription == &automatic) {
      Serial.println((char *)automatic.lastread);
      int automatic_State = atoi((char *)automatic.lastread);
      if(automatic_State==1)
      {
        auto_state=1;
      Serial.println ("Control De Luces Automaticas: Activo..."); 
      }
      if(automatic_State==0)
      {
        auto_state=0;
      Serial.println ("Control De Luces Automaticas: Desactivado...");
      foco.publish("1");
      }
    }

     if (subscription == &Alarma_status) //SWITCH DE ENCENDIDO O APAGADO ALARMA
     {
      Serial.print(F("Alarma status: "));
      Serial.println((char *)Alarma_status.lastread);
      int Alarm_State = atoi((char *)Alarma_status.lastread);
      armed=Alarm_State;
    }


  }
 
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");

}


void blinker()
{
  digitalWrite(alarm_indicator, LOW);                                  
  delay(500);                      
  digitalWrite(alarm_indicator, HIGH);  
  delay(500);        
}

void Noise()
{
  digitalWrite(Relay3, LOW);                                  
  delay(1000);                      
  digitalWrite(Relay3, HIGH);  
  delay(500);        
}
