///////////////////////////////////////////////////////////////////////
//  Project Tilte:  Orange Web matrix display (Iot project)          //
//  Version: 1.0                                                     //
//  Editor: Erwann Caroff                                            //
//  Last uptdate: 16/03/20017                                        //
///////////////////////////////////////////////////////////////////////
/*
  NodeMCU pins:

  -->Matrix 4 modul:
  .CLK -> D6 (14)
  .CS  -> D3 (0)
  .DIN-> D7 (13)
  .GND-> GND
  .VCC-> 3.3V

  -->BMP180 (presure/temperatur/altitude sensor)
  .VCC --> 3.3V / 5V
  .GND --> GND
  .SCL-> D1
  .SDA-> D2

  -->DHT11 (humidity / temperatur)
  .GND (- on Conrad breakout board) -> GND
  .VCC                              -> 3.3V
  .D4 (s on Conrad breakout board)  ->  Din

  -->Microphone (Conrad breakout board)
  .GND (G on Conrad breakout board) -> Gnd
  .VCC (+ on Conrad breakout board) -> 3.3V
  .A0                               -> AD0
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>
#include <time.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Timer.h"
Timer t;
Timer t1;


//Sensor lib
#include <DHT11.h>
#include <Adafruit_Sensor.h>        //Librairie pour le capteur KY 052 (présion atmosphérique) 
#include <Wire.h>;
#include <Adafruit_BMP085.h>;
Adafruit_BMP085 bmp;

//Mqtt librarie
#include <ESPAsyncTCP.h>
#include <AsyncMqttClient.h>

  WiFiClient client;

// NTP Servers:(date/heur)
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = 2;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

//Buffer qui permet de décoder les messages MQTT reçus
char message_buff[100];

long lastMsg = 0;   //Horodatage du dernier message publié sur MQTT
long lastRecu = 0;

AsyncMqttClient mqttClient;
ESP8266WebServer server(80);                             // HTTP server will listen at port 80

//Capteur DHT
int pin = 2;
DHT11 dht11(pin);
// ******************* String form to sent to the client-browser ************************************
char form[3000] = "";

long period;
int offset = 1, refresh = 0;
int pinCS = 0; // Attach CS to this pin, DIN to MOSI and CLK to SCK (cf http://arduino.cc/en/Reference/SPI )
int numberOfHorizontalDisplays = 4;//Nombre de matrice de Led (Horizontal)
int numberOfVerticalDisplays = 1;//Nombre de matrice de Led (Vertical)
String decodedMsg;
#define NoiseLimit 3
float gainFinal = 0;


int a, b, c = 0;
int msgScrol = 0;
char ChipId [20];                               //Chip ID array
int cptChipId = 0;
bool functionLock = 0;
bool bmplock = 0;
int dispInstensity = 0;
bool veille = 0;
# define LengthTabGain 100
float tabGain[LengthTabGain];//Moyenne le bruit sur 10 secondes
int PointeurTabGain = 0;   //Variable qui place les valeurs dans le tableau de niveau de bruit
bool lockStartBySound = 0;
//char SSIDvar[100]=" ";
char bufSSID[100] = "";  
char bufPASS[100] = "";
char bufIntens[2] = "";
char bufspeedScroling[100] = "";
String connexionMode;
int loopCpt=0;
char result[16];
char SourceIdArray[30];



#define secondeConexionTry 10
#define SeuilleMarcheCafetiere 30
#define SensorNumber 12
//La luminosité et l'altitude ne sont pas utilisé, cependant les varaible sont inscites afin de garder une compatiboilité avec la page web  utiliser par la webradio
String sensor[][SensorNumber] =
{
  {"Temp", "Hygro", "PsAt", "Lum", "Bruit", "Alt", "tempCaf", "Message", "Heure", "Date", "IP","RSSI"},
  {"Nc", "Nc", "Nc", "Nc", "Nc", "Nc", "\"Nc\"", "Nc", "Nc", "Nc", "Nc","Nc"},
  {"oC", "%", "hPa", "", "%", "m", "oC", "", "", "", "", ""}
};

Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);

String tape = "Arduino";
int wait = 25; // In milliseconds

int spacer = 2;
int width = 5 + spacer; // Espaces entre caracteres

/*
  handles the messages coming from the webbrowser, restores a few special characters and
  constructs the strings that can be sent to the oled display
*/
void handle_msg() {
  matrix.fillScreen(LOW);
  webPageEdit();
  refresh = 1;
  // Display msg on Oled
  String msg = server.arg("msg");
  Serial.println("Msg: ");
  Serial.println(msg);

  Serial.println(msg);
  decodedMsg = msg;
  // Restore special characters that are misformed to %char by the client browser
  decodedMsg.replace("+", " ");
  decodedMsg.replace("%21", "!");
  decodedMsg.replace("%22", "");
  decodedMsg.replace("%23", "#");
  decodedMsg.replace("%24", "$");
  decodedMsg.replace("%25", "%");
  decodedMsg.replace("%26", "&");
  decodedMsg.replace("%27", "'");
  decodedMsg.replace("%28", "(");
  decodedMsg.replace("%29", ")");
  decodedMsg.replace("%2A", "*");
  decodedMsg.replace("%2B", "+");
  decodedMsg.replace("%2C", ",");
  decodedMsg.replace("%2F", "/");
  decodedMsg.replace("%3A", ":");
  decodedMsg.replace("%3B", ";");
  decodedMsg.replace("%3C", "<");
  decodedMsg.replace("%3D", "=");
  decodedMsg.replace("%3E", ">");
  decodedMsg.replace("%3F", "?");
  decodedMsg.replace("%40", "@");

}

void handle_ConfigRestart()
{
  webPageEdit();
  String ssid = server.arg("ssid");
  Serial.print("Ssid: ");
  Serial.println(ssid);
  String password = server.arg("password");
  Serial.print("Password: ");
  Serial.println(password);

  const int  ssidlength = ssid.length();
  const int  passwordlength = password.length();
  const int totalconfigLength = ssidlength  + passwordlength + 5;

  String stringConfig =  "d/" + ssid + "/" + password + "/" ;
  char arrayConfig [totalconfigLength];

  Serial.print("string config: ");
  Serial.println(stringConfig);

  stringConfig.toCharArray(arrayConfig, totalconfigLength);

  Serial.print("array config: ");
  Serial.println(arrayConfig);
  //eeprom clear
  for (int i = 0; i < totalconfigLength; i++)
  {
    EEPROM.write(i, 0);
  }

  //Write  in eeprom
  for (int i = 0 ; i < totalconfigLength ; i ++)
  {
    EEPROM.write(i, arrayConfig[i]);
  }

  EEPROM.commit();
  Serial.println("Restart..");

  ESP.restart();
}

void handle_Configintensity()
{
  webPageEdit();
  int intensity = server.arg("intensity").toInt();
  String intensityString = String(intensity);
  int intensityLength = intensityString.length();
  //bufIntens = ' ';
  intensityString.toCharArray(bufIntens, intensityLength + 1);

  if (intensity < 16)
  {
    matrix.setIntensity(intensity); // Use a value between 0 and 15 for brightness

    //clear eepom for Disp ligth intensity
    for (int i = 100 ; i < 2 + 100 ; i ++)
    {
      EEPROM.write(i, ' ');
    }

    //Write  in eeprom
    for (int i = 100 ; i < 2 + 100 ; i ++)
    {
      EEPROM.write(i, bufIntens[i - 100]);
    }
    
   //memcpy(bufIntens, buffintensity , sizeof(bufIntens ));
   webPageEdit();

    EEPROM.commit();
  }
}


void handle_configSpeedDisplay()
{
  webPageEdit();
  int speedScroling = server.arg("speedScroling").toInt();

  String speedScrolingString = String(speedScroling);
  int speedScrolingLength = speedScrolingString.length();
  //bufspeedScroling= ' ';
  speedScrolingString.toCharArray(bufspeedScroling, speedScrolingLength + 1);

  if (speedScroling < 1000 & speedScroling > 10)
  {
    wait = speedScroling; // Use a value between 0 and 15 for brightness

    //clear eepom for Disp ligth intensity
    for (int i = 110 ; i < 3 + 110 ; i ++)
    {
      EEPROM.write(i, ' ');
    }

    //Write  in eeprom
    for (int i = 110 ; i < 3 + 110 ; i ++)
    {
      EEPROM.write(i, bufspeedScroling[i - 110]);
    }
    webPageEdit();
    EEPROM.commit();
  }
}

void handle_configStartBySound()
{
  webPageEdit();
  int StartBySound = server.arg("StartBySound").toInt();

  if (StartBySound == 1 | StartBySound == 0)
  {
    lockStartBySound = StartBySound ; // Use a value between 0 and 15 for brightness
    EEPROM.write(120, StartBySound);
    EEPROM.commit();
  }


}

void webPageEdit()
{
// <head><meta http-equiv='refresh' content='5'>
  sprintf ( form,
   "<html>\
  <style>h1{ color: orange;} </style>\  
  <h1>Orange smart display</h1>\
  <head></head>\
  <form action='msg'><p>Entrez un Message <input type='text' name='msg' size=100 > <input type='submit' value='Submit' ></form>\
  <form action='config&restart'><p>Wifi SSID:  <input type='text' name='ssid' size=5> Password: <input type='text' name='password' size=5 autofocus > <input type='submit' value='Enregistre et redemarre'> </form>\
  <p>Actuellement configure : ssid: %s, Password %s</p>\
  <form action='configintensity'><p>Luminosite ((-)0-->15(+)): <input type='text' name='intensity' size=5 autofocus > <input type='submit' value='submit'></form>\
  <p>Actuellement configure : Luminosite: %s</p>\
  <form action='configSpeedDisplay'><p> Vitesse de defilement ((-)100-->11(+)): <input type='text' name='speedScroling' size=5 autofocus> <input type='submit' value='submit'></form>\
  <p>Actuellement configure : Vitesse defilement: %s</p>\
  <form action='configStartBySound'><p> Mise en veille de l'afficheur (0=Marche forcee)(1=Veille automatique) <input type='text' name='StartBySound' size=5 autofocus> <input type='submit' value='submit'></form>\
  <a href='https://smart-display.kmt.orange.com/' target='_blank'>Lien page web</a>\
  <p></p>\
  <form><img src='https://upload.wikimedia.org/wikipedia/commons/c/c8/Orange_logo.svg'></form>\
  </html>"
   , bufSSID,bufPASS,bufIntens,bufspeedScroling);


  server.on("/", []() {
 server.send(200, "text/html", form);
  });
  server.send(200, "text/html", form);
//  Serial.print("ssid : ");
//  Serial.println(String(bufSSID));
//  
//  Serial.print("form1: ");
//  Serial.println(form);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("** Connected to the broker **");
  //  Serial.print("Session present: ");
  //  Serial.println(sessionPresent);
  //  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  //  Serial.print("Publishing at QoS 1, packetId: ");
  //  Serial.println(packetIdPub1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("** Disconnected from the broker **");
  Serial.println("**  Reconnecting to MQTT... **");
  mqttClient.connect();
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("** Subscribe acknowledged **");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("** Unsubscribe acknowledged **");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("** Publish received **");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("** Publish acknowledged **");
  //  Serial.print("  packetId: ");
  //  Serial.println(packetId);
}

void otaConfiguration()
{
  // Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  //ArduinoOTA.setHostname("Smart_display");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*                                                                                      SETUP                                                               */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  //Initialize sound table
  tabGain[LengthTabGain] = ' ';
  veille = 0;
  //ESP.wdtDisable();                               // used to debug, disable wachdog timer,
  Serial.begin(115200);                           // full speed to monitor

  matrix.fillScreen(LOW); //Clear matrix sceen

  //BMP
  Wire.begin(4, 5);

  //Get Chip ID
  String str = String(ESP.getChipId());               //Get the chip id
  str.toCharArray(ChipId, 20);                        //Put the Chip Id in a array

  for (int i = 0 ; i < sizeof(ChipId) ; i++)
  {
    if (isDigit(ChipId[i]))
    {
      cptChipId = cptChipId + 1;
    }
  }

  EEPROM.begin(512);

  // Adjust to your own needs (à configurer selon la taille du nombre de matrcie)
  matrix.setPosition(0, 3, 0); // The first display is at <0, 7>
  matrix.setPosition(1, 2, 0); // The second display is at <1, 0>
  matrix.setPosition(2, 1, 0); // The third display is at <2, 0>
  matrix.setPosition(3, 0, 0); // And the last display is at <3, 0>

  //eeprom storage:(d/ ;     ssid     ; / ;     pasword ; /)


  int p = 0;
  int h = 0;
  int g = 0;
  int j = 0;

  //read and Store data from eepom
  for (int i = -1; i <= 99; i++)
  {
    if (EEPROM.read(i) == '/')
    {
      if (p == 2)
      {
        i = 99;
      }
      else
      {
        p = p + 1;
      }
    }
    if (p == 1)
    {
      if (EEPROM.read(i) == '/')
      {

      }
      else
      {
        bufSSID[g] = EEPROM.read(i);
        g = g + 1;
      }

    }
    else if (p == 2)
    {
      if (EEPROM.read(i) == '/')
      {

      }
      else
      {
        bufPASS [h] = EEPROM.read(i);
        h = h + 1;
      }
    }
  }

  //Read ssid and wifi pasword in eeprom
  bufIntens[0] = EEPROM.read(100);
  bufIntens[1] = EEPROM.read(101);

  //Read speed scroling in eeprom

  bufspeedScroling[0] = EEPROM.read(110);
  bufspeedScroling[1] = EEPROM.read(111);
  bufspeedScroling[2] = EEPROM.read(112);

  //Read StartBySound flag in eeprom
  lockStartBySound  = EEPROM.read(120);


  Serial.print("Ssid conect: ");
  Serial.println(String(bufSSID));
  
//  SSIDvar = bufSSID;
  
  Serial.print("Pasword conect: ");
  Serial.println(String(bufPASS));
  dispInstensity = atoi(bufIntens);
  Serial.print("Int Disp: ");
  Serial.println(dispInstensity);
  matrix.setIntensity(dispInstensity); // Use a value between 0 and 15 for brightness
  if (atoi(bufspeedScroling) >= 11 & atoi(bufspeedScroling) <= 100)
  {
    wait = atoi(bufspeedScroling);
  }
  Serial.print("Speed scroling: ");
  Serial.println(wait);

  int cpt = 0;
  bool ExitWhile = 0;

  WiFi.begin(bufSSID, bufPASS);                         // Connect to WiFi network
  while ( ExitWhile == 0 )
  { // Wait for connection

    delay(1000);
    Serial.print("Connexion Try: ");
    Serial.print(cpt);
    Serial.println("/10");
    cpt = cpt +  1;

    if (WiFi.status() == WL_CONNECTED)
    {
      functionLock = 1;
      Serial.println("Gateway connexion OK");
      GatewayServerMode();
      functionLock = 0;
      ExitWhile = 1;
      Serial.println("Unlocked");
    }

    if (cpt > secondeConexionTry) //(After 10s, Ap conexion mode)
    {
      Serial.println("Gateway connexion fail");
      ApMode();
      Serial.println("Mode: Access point");
      ExitWhile = 1;
      functionLock = 1;
      Serial.println("locked Mode");
    }
  }

}


void GatewayServerMode()
{

  connexionMode = "Gateway";
  server.on("/", []() {
    webPageEdit();
  });


  server.on("/msg", handle_msg);                  // And as regular external functions:
  server.begin();                                 // Start the server

  sprintf(result, "%3d.%3d.%1d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  
  decodedMsg = result;
  Serial.println("WebServer ready!   ");

  Serial.println(WiFi.localIP());                 // Serial monitor prints localIP

  //Initialisation serveur liveobject
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer("liveobjects.orange-business.com", 1883);


  memset(SourceIdArray,'\0',30);
  String SourceId = "SmartDisplay:" + String(ChipId);
  int SourceIdLength = SourceId.length()+1;  
  SourceId.toCharArray(SourceIdArray, SourceIdLength);
  
  
  Serial.print("Source: ");
  Serial.println(String(SourceIdArray));
  mqttClient.setKeepAlive(20).setCleanSession(false).setWill("dev/data", 0, true, "no").setCredentials("json+device", "7ef7949fb37c454aa3087f63c276573d").setClientId(SourceIdArray);
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();


  Serial.print("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);

  //Configuration of wifi uploading
  otaConfiguration();

  server.on("/msg", handle_msg);
  server.on("/config&restart", handle_ConfigRestart);
  server.on("/configintensity", handle_Configintensity);
  server.on("/configSpeedDisplay", handle_configSpeedDisplay);
  server.on("/configStartBySound", handle_configStartBySound);
  
  //Set timer for publisch data on liveobject(every 30s)
  t.every(30000, mqttPublish);
  
  sensorRead();
}

void ApMode()
{
   connexionMode = "AccesPoint";
  /* Set these to your desired credentials. */
  const char *ssid = "SmartDisplay";
  //const char *password = "123456789";
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid);

  server.on("/", []() {
    webPageEdit();
  });

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/msg", handle_msg);
  server.on("/config&restart", handle_ConfigRestart);
  server.on("/configintensity", handle_Configintensity);
  server.on("/configSpeedDisplay", handle_configSpeedDisplay);
  server.on("/configStartBySound", handle_configStartBySound);
  server.begin();
  Serial.println("HTTP server started");

  //Configuration of wifi uploading
  otaConfiguration();
}


// Déclenche les actions à la réception d'un message
// D'après http://m2mio.tumblr.com/post/30048662088/a-simple-example-arduino-mqtt-m2mio
void callback(char* topic, byte* payload, unsigned int length) {

  int i = 0;

  Serial.println("Message recu =>  topic: " + String(topic));
  Serial.print(" | longueur: " + String(length, DEC));

  // create character buffer with ending null terminator (string)
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';

  String msgString = String(message_buff);

  Serial.println("Payload: " + msgString);

  if ( msgString == "ON" ) {
    //Do something
  } else {
    //Do something
  }
}
////////////////////////////////////////////////////////////          Sensor Read           ////////////////////////////////////
void sensorRead()
{
  # define offsetHumid 30
  float humidite = 0, temperatur = 0;
  int temp = 0, humi = 0, PsAtmo = 0, tempCaf = 0, altitude = 0;


  Serial.println();
  Serial.print("Sensor values: ");

  dht11.read(humidite, temperatur);
  temp = temperatur;//Convertion float to int
  Serial.print("Temperature:");
  Serial.print(temp);
  Serial.print("*C");

  Serial.print(" | humidity:");
  humi = humidite + offsetHumid;
  Serial.print(humi);
  Serial.print("%");

  //Lock bmp read (avoid arduino reset if no sensor connected)
  if (!bmp.begin())
  {
    Serial.print(" | BMP180 / BMP085 introuvable ! Verifier le branchement ");
    bmplock = 1;
  }
  if ( bmplock  == 0)
  {
    Serial.print(" | TemperatureCaf: ");
    tempCaf = bmp.readTemperature();
    Serial.print(tempCaf);
    Serial.print(" *C");

    Serial.print(" | Pression = ");
    PsAtmo = bmp.readPressure() / 100;
    Serial.print(PsAtmo);
    Serial.print(" hPa");

    Serial.print(" | Altitude = ");
    altitude = bmp.readAltitude();
    altitude = altitude * -1;
    Serial.print(altitude);
    Serial.print(" metres");
  }


  Serial.print(" | Bruit = ");
  Serial.print(gainFinal);


  //RAZ du tabeau de valeurs
  for (int i = 0 ; i < SensorNumber ; i++) //Scan de la structure pour trouver la zone à modifier
  {
    sensor [1] [i] = ' ';
  }


  // digital clock display of the time
  getTime();
  Serial.print(" | Time: ");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print("| Date: ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());

  sprintf(result, "%3d.%3d.%1d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

  Serial.print(" | Ip:");
  Serial.print(String(result));

  Serial.print(" | rssi:") ;
  Serial.println( WiFi.RSSI());

  //Enregistrement des valeurs dans le tableau
  sensor [1] [0] =  String(temp);
  sensor [1] [1] = String (humi);
  sensor [1] [2] = String(PsAtmo) ;
  sensor [1] [3] =  '"' + String ("NC") + '"' ;
  sensor [1] [4] = String(gainFinal) ;
  sensor [1] [5] = String(altitude) ;
  sensor [1] [6] = String(tempCaf) ;
  sensor [1] [7] = '"' + String(decodedMsg) + '"';
  sensor [1] [8] = '"' + String(hour()) + ":" + String(minute()) + '"';
  sensor [1] [9] = '"' + String(day()) + "/" + String(month()) + "/" + String(year()) + '"';
  sensor [1] [10] = '"' + String(result) + '"' ; 
  sensor [1] [11] = '"' + String(WiFi.RSSI()) + '"' ;
}

void getNoise()
{
  const int sampleWindow = 20; // Sample window width in mS (50 mS = 20Hz)
  unsigned int sample;
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level
  int signalMax = 0;
  int signalMin = 676;
  double gainDynamique = 0;
  double volte = 0;
  int i = 0;

# define sensibilite 3 //On sensibilise le micro de 10%


  //Annalyse du gain
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(A0);
    // Serial.print("ADC: ");
    //Serial.println(sample );


    if (sample < 676 )  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }

  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  peakToPeak = peakToPeak * sensibilite;  //On augmente la sensibilité du capteur

  volte = (peakToPeak * 3.3) / 676 ;  // convert to volts
  gainDynamique = (volte * 100) / 3.3;  //On convertie en % (saturation du micro adc --> 676  = gainDynamique --> 100%)

  if (gainDynamique > 101)//On limite le gainDynamique en sturation à 100%
  {
    gainDynamique = 100;
  }

  PointeurTabGain = PointeurTabGain + 1;

  if (PointeurTabGain == LengthTabGain )
  {
    PointeurTabGain = 0;
  }

  if (gainDynamique < 1)//On supprime le % en dessous de 2.23% qui du à l'impréssision du capteur
  {
    gainDynamique = 0.00;
  }
  tabGain[PointeurTabGain] = gainDynamique;

  //Take the higher noise value
  gainFinal = 0;
  for (int i = 0 ; i < LengthTabGain ; i++)
  {
    if (gainFinal < tabGain[i])
    {
      gainFinal = tabGain[i] ;
    }
  }

  //no Noise => no one in the room => standby display
  if (gainFinal > NoiseLimit)
  {
    veille = 0;
  }
  else
  {
    veille = 1;
  }

}

time_t prevDisplay = 0; // when the digital clock was displayed
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                                      mqttPublish
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mqttPublish()
{
  if (functionLock == 0)
  {
# define arrayLenght 300

    String strucDataAndOnlineCheck = "{\"s\":\"urn:lo:nsid:smartDisplay:ChId!Data\",\"m\":\"smartDisplay_V1_0\",\"v\":{ ";
    char buf1[arrayLenght];
    char buf2[arrayLenght];


    //////////////////////////////////////////////////////////////////////////
    strucDataAndOnlineCheck.toCharArray(buf1, arrayLenght);
    strncpy(buf2, buf1, arrayLenght);

    for (int i = 0 ; i < sizeof(buf1) ; i++) //Scan de la structure pour trouver la zone à modifier
    {
      if ((buf1[i] == 'C') & (buf1[i + 1] == 'h') & (buf1[i + 2] == 'I') & (buf1[i + 3] == 'd')) //Détéction de la zone à modifier
      {

        for (int l = 0 ; l < (sizeof(buf1) - i) ; l++) //1ere étape: Ecrire l'ID dans la structure
        {
          buf2 [i + l] = char(ChipId[l]);
        }


        for (int j = 0 ; j < 49 ; j++)//2eme étape: Ajouter le reste de la structure
        {
          buf2 [i + j + cptChipId ]  = buf1 [i + j + 4];
        }
      }
    }
    strucDataAndOnlineCheck = String(buf2);  //Réintégration de la structure modifier dans ca chaine de catactere par défaut

    memset(buf1, 0, sizeof(buf1));
    memset(buf2, 0, sizeof(buf2));
    /////////////////////////////////////////////////////

    for (int h = 0 ; h < SensorNumber ; h++)
    {
      strucDataAndOnlineCheck = strucDataAndOnlineCheck + '"';
      strucDataAndOnlineCheck = strucDataAndOnlineCheck + sensor[0][h];
      strucDataAndOnlineCheck = strucDataAndOnlineCheck + '"';
      strucDataAndOnlineCheck = strucDataAndOnlineCheck + ':';
      strucDataAndOnlineCheck = strucDataAndOnlineCheck + sensor[1][h];

      if (h != SensorNumber - 1)
      {
        strucDataAndOnlineCheck = strucDataAndOnlineCheck + ",";
      }
    }

    strucDataAndOnlineCheck = strucDataAndOnlineCheck + "},";
    strucDataAndOnlineCheck.toCharArray(buf1, arrayLenght) ;


    ///////////////////////////////////////////////////////////////////////////
    String stringTag = String(buf1);
    String tags ="\"tags\":[\"erwann\"]}";
    int tagsLength = tags.length();
    stringTag = stringTag + tags;

    Serial.print("tags: ");
    Serial.println(tags);
    
    stringTag.toCharArray(buf1, arrayLenght+tagsLength ) ;
    
    mqttClient.publish ("dev/data", 1, true, buf1) ; //Envoie des données

    Serial.print("Json Envoye:");
    Serial.println(String(buf1));
    memset(buf1, 0, sizeof(buf1));                     //Clear buffer
  }

}
/////////////////////////////////////////////////////////////////////       Get time        ////////////////////////////////////////
void getTime()
{
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
    }
  }
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void matrixDisplay ()
{
  int lengthbufName = sensor[0] [msgScrol].length() + 1;
  int lengthbufValue = sensor[1] [msgScrol].length() + 1;
  int lengthbufUnit = sensor[2] [msgScrol].length() + 1;
  int TotalLength = lengthbufValue + lengthbufName + lengthbufUnit;

  char buf [TotalLength];

  for (int j = 0; j < TotalLength; j++)
  {
    buf[j] = ' ';
  }

  char bufName[lengthbufName];
  sensor [0] [msgScrol].toCharArray(bufName, lengthbufName);


  char bufValue[lengthbufValue];
  sensor [1] [msgScrol].toCharArray(bufValue, lengthbufValue);

  char bufCleanValue [lengthbufValue];
  int j = 0;

  //Erase the " or space into data values for display
  for ( int i = 0 ; i < lengthbufValue; i++ )
  {
    if (bufValue[i] == '"') //Avoid the " in the display msg
    {
    }
    else
    {
      bufCleanValue [j] = bufValue[i];
      j = j + 1;
    }
  }

  char bufUnit[lengthbufUnit];
  sensor [2] [msgScrol].toCharArray(bufUnit, lengthbufUnit);



  /////Liste des infos à ne pas afficher sur l'écran

  //  0:Températur ambiante
  //  1:Hygrométrie
  //  2:Ps atmo
  //  3:Luminosité -->Ne pas afficher
  //  4:Bruit-->Ne pas afficher
  //  5:Altitude -->Ne pas afficher
  //  6:Temp Cafetiere>Ne pas afficher
  //  7:Méssage
  //  8:Heure
  //  9:Date
  //  10:IP-->Ne pas afficher
  //  11:RSSI-->Ne pas afficher



  bool  lock = 0;
  while (lock != 1)
  {
    msgScrol =  msgScrol + 1;
    //msgScrol ==6 |
    if ( msgScrol == 3 | msgScrol == 4 | msgScrol == 5 | msgScrol == 10| msgScrol == 11)
    {
      lock = 0;
    }
    else
    {
      lock = 1;
    }

  }

  if (msgScrol >= SensorNumber)
  {
    msgScrol = 0;
  }

  //Buiding à the string to display (Name of value : value (unit))
  String stringbuf = String(bufName) + ':' + String(bufCleanValue) + String(bufUnit);
  stringbuf.toCharArray( buf, TotalLength);


  int OffsetSizeTotalLength = TotalLength - 2; //Diminution des incéments inutiles (réduit le temp entre chaque méssage)

  for ( int i = 0 ; i < width * OffsetSizeTotalLength + matrix.width() - 0 - spacer; i++ )
  {
    if (refresh == 1) i = 0;
    refresh = 0;
    matrix.fillScreen(LOW);
    server.handleClient();                        // checks for incoming messages
    int letter = i / width;
    int x = (matrix.width() - 1) - i % width;
    int y = (matrix.height() - 8) / 2; // center the text vertically

    while ( x + width - spacer >= 0 && letter >= 0 ) {
      if ( letter < sizeof(buf) )
      {
        matrix.drawChar(x, y,  buf[letter], HIGH, LOW, 1);
        server.handleClient();                        // checks for incoming messages
        ArduinoOTA.handle();                          // checks for incoming uptdate
      }
      letter--;
      x -= width;
    }
    //(rotation des afficheurs)
    matrix.setRotation(0, 3);
    matrix.setRotation(1, 3);
    matrix.setRotation(2, 3);
    matrix.setRotation(3, 3);

    matrix.write(); // Send bitmap to display
    delay(wait);
  }
}

void DisplayAccesPointMsg()
  {
    String DeconexionMsg ="Display is disconnected from wifi, please set up wifi settings! ";
    int DeconexionMsgLength=DeconexionMsg.length();
    char DeconexionMsgArray[DeconexionMsgLength];
    DeconexionMsg.toCharArray( DeconexionMsgArray, DeconexionMsgLength);
    
      for ( int i = 0 ; i < width * DeconexionMsgLength + matrix.width() - 0 - spacer; i++ )
      {
        if (refresh == 1) i = 0;
        refresh = 0;
        matrix.fillScreen(LOW);
        server.handleClient();                        // checks for incoming messages
        int letter = i / width;
        int x = (matrix.width() - 1) - i % width;
        int y = (matrix.height() - 8) / 2; // center the text vertically
    
        while ( x + width - spacer >= 0 && letter >= 0 ) {
          if ( letter < sizeof(DeconexionMsgArray) )
          {
            matrix.drawChar(x, y,  DeconexionMsgArray[letter], HIGH, LOW, 1);
            server.handleClient();                        // checks for incoming messages
            ArduinoOTA.handle();                          // checks for incoming uptdate
          }
          letter--;
          x -= width;
        }
        //(rotation des afficheurs)
        matrix.setRotation(0, 3);
        matrix.setRotation(1, 3);
        matrix.setRotation(2, 3);
        matrix.setRotation(3, 3);
    
        matrix.write(); // Send bitmap to display
        delay(wait);
      }
  }
  void getLiveObjectData()
  {
  
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                          Loop                                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(void)
{
  ArduinoOTA.handle();
  server.handleClient(); // checks for incoming messages
  Serial.print("conexion staus: ") ;
  Serial.println(WiFi.status());


  if(connexionMode== "Gateway")
  {
      if (veille == 0 | lockStartBySound == 0)
      {
        matrixDisplay();
      }
      else
      {
        matrix.fillScreen(LOW);
        delay(10);
      }
      t.update();
      sensorRead();
      getNoise();
      getLiveObjectData();
  }
  else
  {
       DisplayAccesPointMsg();
       loopCpt = loopCpt +1;
       
       //wait (5min) before restart (1 loop = 10sec)
       if(loopCpt == 30)
       {
          ESP.restart();
       }
  }
}


