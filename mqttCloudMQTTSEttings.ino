/**
 * \file
 *       ESP8266 MQTT Bridge example
 * \author
 *       Tuan PM <tuanpm@live.com>
 */
#include <SoftwareSerial.h>
#include <espduino.h>
#include <mqtt.h>


#include "DHT.h"

#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define NEOPIXELPIN 7



#define DHTPIN 11    // what pin we're connected to


#define DHTTYPE DHT11   // DHT 11 


#define INTCOUNTER 5
boolean oneTimeSetup=false;
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial debugPort(5, 6); // RX, TX
ESP esp(&Serial, &debugPort, 2);
MQTT mqtt(&esp);
boolean wifiConnected = false;

char* endpoints[]={"/CG/room/1/light1", "/CG/room/1/light2","/CG/room/1/light3","/CG/room/1/light4"};

//char* endpoints[]={"/CG/room/1/light1"};
char* master="/CG/room/1/master";
int sizeOfEndpointArray=sizeof(endpoints)/sizeof(char*);
char humidity[10];
char temperature[10];
int pinMapping[]={7,8,9,10};
boolean isSetup=false;
int LEDCount=8;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LEDCount, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);

//String serverURL = "m11.cloudmqtt.com";
String serverURL = "iot.eclipse.org";
//int brokerPort=17478;
int brokerPort=1883;
//char* brokerUsername= "uname";
//char* brokerPassword="pwd";
//char* clientID="ArduinoClient";

char* pubTempTopic="/CG/room/1/temperature";
char* pubHumidityTopic="/CG/room/1/humidity";	

//char* pubTempTopic="/CG";
//char* pubHumidityTopic="/CG";
//String pubHumidityTopic=API_key+Project_ID+"/sensor"+humiditySensorID;	

int counter=0;
void wifiCb(void* response)
{
  uint32_t status;
  RESPONSE res(response);

  if(res.getArgc() == 1) {
    res.popArgs((uint8_t*)&status, 4);
    if(status == STATION_GOT_IP) {
      debugPort.println("WIFI CONNECTED");
      mqtt.connect("iot.eclipse.org", 1883);
      wifiConnected = true;
      //or mqtt.connect("host", 1883); /*without security ssl*/
    } else {
      wifiConnected = false;
      mqtt.disconnect();
    }
    
  }
}

void mqttConnected(void* response)
{
  debugPort.println("Connected");
    
    debugPort.print("Subscribing to ");
    debugPort.print(sizeOfEndpointArray);
    debugPort.println(" topics");
    for(int i=0;i<sizeOfEndpointArray;i++)
    {
     
     mqtt.subscribe(endpoints[i]); 
      
    } 
   mqtt.subscribe(master);
   mqtt.publish(endpoints[0],"on"); 
   
    debugPort.print("Subscribed to ");
    debugPort.print(sizeOfEndpointArray);
    debugPort.println(" topics");
  

  
    
    

}
void mqttDisconnected(void* response)
{

}
void mqttData(void* response)
{
  RESPONSE res(response);

  debugPort.print("Received: topic=");
  String topic = res.popString();
  debugPort.println(topic);

  debugPort.print("data=");
  String data = res.popString();
  debugPort.println(data);
  
  doAction(topic, data);
  

}
void mqttPublished(void* response)
{

}
void setup() {
  
  setupTimer();
  dht.begin();
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
    
  Serial.begin(19200);
  debugPort.begin(19200);
  for (int i=0; i<4; i++)
  {
     pinMode(pinMapping[i], OUTPUT); 
    
  }
  
 /* for (int i=0; i<2; i++)
  {
     digitalWrite(pinMapping[i], LOW); 
     delay(500);
     digitalWrite(pinMapping[i], HIGH); 
     delay(500);
    
  }*/
  debugPort.println("Bringing up ESP!"); 
  esp.enable();
  delay(500);
  debugPort.println("Resetting up ESP!"); 
  esp.reset();
  delay(500);
  while(!esp.ready())
  {
   debugPort.println("ESP Not Ready!"); 
  } 

  debugPort.println("ARDUINO: setup mqtt client");
  
  if(!mqtt.begin("Arduino","uname","pwd", 120, 1)) {
    debugPort.println("ARDUINO: fail to setup mqtt");
    while(1);
  }


  debugPort.println("ARDUINO: setup mqtt lwt");
  mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");

/*setup mqtt events */
  mqtt.connectedCb.attach(&mqttConnected);
  mqtt.disconnectedCb.attach(&mqttDisconnected);
  mqtt.publishedCb.attach(&mqttPublished);
  mqtt.dataCb.attach(&mqttData);

  /*setup wifi*/
  debugPort.println("ARDUINO: setup wifi");
  esp.wifiCb.attach(&wifiCb);

  esp.wifiConnect("ssid","pwd");


  debugPort.println("ARDUINO: system started");
  //setTime(8,29,0,1,1,11); // set time to Saturday 8:29:00am Jan 1 2011
  //Alarm.timerRepeat(15, Repeats); 
  
  
  
}

void loop() {
  esp.process();
  
  if(wifiConnected) {
    if(counter==0 && oneTimeSetup)
    {
      oneTimeSetup=false;
      publishDHTData();
      
    }  
    
  }
  
  
  
}

     



boolean doAction(String light, String data)
{
    data.toLowerCase();
    //debugPort.println("Data: "+data);
   if(data.equals("on"))
  {
     //debugPort.println("Inside On");
    for(int i=0;i<sizeOfEndpointArray;i++)
    {
     //debugPort.println(light+ " : " +endpoints[i]);
      if(light==endpoints[i] || light==master)
      {
         //digitalWrite(pinMapping[i], HIGH); 
      //   debugPort.println(pinMapping[i]);
          setLEDColor(i, 0xFF11dd);

      } 

      
    } 
     
   
   
  }
  
  if(data.equals("off"))
  {
   //debugPort.println("Inside Off");
    
    for(int i=0;i<sizeOfEndpointArray;i++)
    {
     //debugPort.println(light+ " : " +endpoints[i]);
      if(light==endpoints[i] || light==master)
      {
         //digitalWrite(pinMapping[i], LOW); 
         //debugPort.println(pinMapping[i]);
         setLEDColor(i, 0x000000);

      } 
      

      
    } 
      
      
  }
  
  
  
  
  
 
  
  
}




void publishDHTData()
{
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    debugPort.println("Failed to read from DHT sensor!");
    return;
  }
  // DISPLAY DATA
   //debugPort.println(h);
  //Serial.print(",\t");
  //Serial.println(DHT.temperature, 1);
 // itoa(h,humidity,10);
 // itoa(t,temperature,10);
 
 //snprintf(humidity,sizeof(humidity), "%f", h);
 //snprintf(temperature,sizeof(temperature), "%f", t);
 
 dtostrf(h , 2, 2, humidity); 
  dtostrf(t , 2, 2, temperature); 

//strcat(humidity,"\u0020\u0025");
//strcat(temperature, "\u0020\u00B0\u0043");
  //mqtt.publish("/room/1/humidity",humidity);
  mqtt.publish(pubTempTopic,temperature);
   mqtt.publish(pubHumidityTopic, humidity);
   debugPort.print("humidity: ");
  debugPort.println(humidity);
   debugPort.print("temperature: ");
  debugPort.println(temperature);
  
  

  
 
//digitalWrite(5, digitalRead(5) ^ 1); // toggle LED pin 
  
}


void setupTimer()
{
//pinMode(LEDPIN, OUTPUT);

// initialize Timer1
cli();         // disable global interrupts
TCCR1A = 0;    // set entire TCCR1A register to 0
TCCR1B = 0;    // set entire TCCR1A register to 0

// enable Timer1 overflow interrupt:
TIMSK1 |= (1 << TOIE1);
// Preload with value 3036
//use 64886 for 100Hz
//use 64286 for 50 Hz
//use 34286 for 2 Hz
TCNT1=65536;
// Set CS10 bit so timer runs at clock speed: (no prescaling)
TCCR1B =0x05; // Sets bit CS12 in TCCR1B
// This is achieved by shifting binary 1 (0b00000001)
// to the left by CS12 bits. This is then bitwise
// OR-ed into the current value of TCCR1B, which effectively set
// this one bit high. Similar: TCCR1B |= _BV(CS12);
//  or: TCCR1B= 0x04;

// enable global interrupts:
sei();
}

ISR(TIMER1_OVF_vect)
{
//digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  debugPort.print("counter:");
  debugPort.println(counter);
  if(counter<INTCOUNTER)
  {
    counter=counter+1;
    oneTimeSetup=true;
  } 
  else
  {
    
    counter=0;
    
   
   
  } 
}


void setLEDColor(uint8_t index,uint32_t color)
{
 strip.setPixelColor(index, color);
 strip.show();  
  
}
