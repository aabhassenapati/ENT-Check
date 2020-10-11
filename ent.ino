
// #include <utility/wifi_drv.h>


#include "Firebase_Arduino_WiFiNINA.h"
//#include "DHT.h"
#include <SPI.h>
#include <WiFiNINA.h>


#define FIREBASE_HOST "teamrio-backend.firebaseio.com"
#define FIREBASE_AUTH "ur auth token"
#define WIFI_SSID "GUSEC"
#define WIFI_PASSWORD ""

//Define Firebase data object
FirebaseData firebaseData;
float RS_gas = 0;
float ratio = 0;
float sensorValue = 0;
float sensor_volt = 0;
int R0 = 2450;
void setup()
{

  Serial.begin(9600);
 //delay(100);
  Serial.println();

  Serial.print("Connecting to Wi-Fi");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  //Provide the autntication data
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);

//WiFiDrv::pinMode(25, OUTPUT);  //GREEN
//WiFiDrv::pinMode(26, OUTPUT);  //RED
//WiFiDrv::pinMode(27, OUTPUT);  //BLUE
}

void loop()
{
  
  String path = "/sunhacks-proj";

   R0 = 2450;
   sensorValue = analogRead(A1);
   sensor_volt = (sensorValue*3.3)/4096;
   RS_gas = (3.3-sensor_volt)/sensor_volt;
   ratio = RS_gas/R0; //Replace R0 with the value found using the sketch above
   float x = 1538.46 * ratio;
   float ppm = pow(x,-1.709);
    Serial.println(ppm);
        Firebase.setFloat(firebaseData, path + "/ppm(MQ7)", ppm);
   sensorValue = analogRead(A2);
   sensor_volt = (sensorValue*3.3)/4096;
   RS_gas = (3.3-sensor_volt)/sensor_volt;
   ratio = RS_gas/R0; //Replace R0 with the value found using the sketch above
   x = 1538.46 * ratio;
   float ppm2 = pow(x,-1.709);
    Serial.println(ppm2);
    Firebase.setFloat(firebaseData, path + "/ppm(MQ4)", ppm2); 
  if (ppm>=0.23)
  {
//  WiFiDrv::digitalWrite(26, HIGH); // for full brightness
//    WiFiDrv::digitalWrite(25, LOW); // for full brightness
//    
  }
  else 
  {
//    WiFiDrv::digitalWrite(25, HIGH); // for full brightness
//    WiFiDrv::digitalWrite(26, LOW); // for full brightness
//  
  }
  
   
}
