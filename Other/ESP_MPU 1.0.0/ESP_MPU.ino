#include <ESP8266WiFi.h>
#include <Wire.h>
#include "MPU9250.h"


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);

int status;

#define SendKey 0  //Button to send data Flash BTN on NodeMCU

int port = 8888;  //Port number1
WiFiServer server(port);

//Server connect to WiFi Network
const char *ssid = "FUM_WIMUx";  //Enter your wifi SSID
const char *password = "12345xyz";  //Enter your wifi Password

int a=0,count=0,sample_time=20;
unsigned int count1=0;
char str[100];
float t1=0,t2=0;
byte bytes[4];
int buff;


void MPU_read(void);
//=======================================================================
//                    Power on setup
//=======================================================================
void setup() 
{

    Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  
  pinMode(SendKey,INPUT_PULLUP);  //Btn to send data
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //Connect to wifi
 
  // Wait for connection  
  Serial.println("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    Serial.print(".");
    delay(1024);
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  
  server.begin();
  Serial.print("Open Telnet and connect to IP:");
  Serial.print(WiFi.localIP());
  Serial.print(" on port ");
  Serial.println(port);
  Serial.print("sample time (ms): ");
  Serial.println(sample_time);
} 

//=======================================================================
//                    Loop
//=======================================================================

void loop() 
{
  WiFiClient client = server.available();
  if (client) {
    if(client.connected())
    {
      Serial.println("Client Connected");
    } 
          while(client.connected()){
            t1=millis();
            MPU_read();      
            //Send Data to connected client
            client.write(bytes, 4);
            t2=millis();
//            Serial.println((t2-t1));
            if((t2-t1) < sample_time)
              delay(sample_time-(t2-t1));
            else
              continue;
              
          }
    client.stop();
    Serial.println("Client disconnected"); 
    count1=0;  
  }
}
//

void MPU_read() {
   IMU.readSensor();
// Serial.print(IMU.getAccelX_mss(),6);
// Serial.print("\n");
//  


count1=count1+sample_time; 
sprintf(str,"%i,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f;",count1,IMU.getAccelX_mss(),\
          IMU.getAccelY_mss(),IMU.getAccelZ_mss(),IMU.getGyroX_rads(),\
          IMU.getGyroY_rads(),IMU.getGyroZ_rads(),IMU.getMagX_uT(),\
          IMU.getMagY_uT(),IMU.getMagZ_uT());

  //  buff = 1 * 2147483647;
  //  Serial.print(sizeof(buff));
  //  Serial.print("\n");
  //  Serial.print(buff);
  //  Serial.print("\n");
  //  Serial.print(buff, BIN);
  //  Serial.print("\n");
  //    bytes[0] = (buff >> 24) & 0xFF;
  //    bytes[1] = (buff >> 16) & 0xFF;
  //    bytes[2] = (buff >> 8) & 0xFF;
  //    bytes[3] = (buff) & 0xFF;
  //  Serial.print("\n");
  //  //Serial.print(bytes,BIN);
  //  Serial.print("\n");
  //  Serial.print("\n");
  //  Serial.print("\n");

  //  //  //          buff=int(gValue.x*10000);
  //  //  //          Serial.print(gValue.x);
  //  //  //          Serial.print("\n");
  //
  //  bytes[0] = 255;
  //
    buff = count1;
//    buff = 1 * -j;
    bytes[1] = (buff >> 24) & 0xFF;
    bytes[2] = (buff >> 16) & 0xFF;
    bytes[3] = (buff >> 8) & 0xFF;
    bytes[4] = (buff) & 0xFF;
   //
//      buff = IMU.getAccelX_mss() * j;
// //    buff = 1 * -j;
//       bytes[5] = (buff >> 24) & 0xFF;
//       bytes[6] = (buff >> 16) & 0xFF;
//       bytes[7] = (buff >> 8) & 0xFF;
//       bytes[8] = (buff) & 0xFF;
  
//     buff = IMU.getAccelY_mss() * j;
//     //buff = 1 * -j;
//       bytes[9] = (buff >> 24) & 0xFF;
//       bytes[10] = (buff >> 16) & 0xFF;
//       bytes[11] = (buff >> 8) & 0xFF;
//       bytes[12] = (buff) & 0xFF;
  //
  //
  //  buff = gValue.z * j;
  //  //buff = 1 * -j;
  //    bytes[13] = (buff >> 24) & 0xFF;
  //    bytes[14] = (buff >> 16) & 0xFF;
  //    bytes[15] = (buff >> 8) & 0xFF;
  //    bytes[16] = (buff) & 0xFF;
  //
  //  buff = gyr.x * j;
  //  //buff = 1 * -j;
  //    bytes[17] = (buff >> 24) & 0xFF;
  //    bytes[18] = (buff >> 16) & 0xFF;
  //    bytes[19] = (buff >> 8) & 0xFF;
  //    bytes[20] = (buff) & 0xFF;
  //
  //  buff = gyr.y * j;
  //  //buff = 1 * -j;
  //    bytes[21] = (buff >> 24) & 0xFF;
  //    bytes[22] = (buff >> 16) & 0xFF;
  //    bytes[23] = (buff >> 8) & 0xFF;
  //    bytes[24] = (buff) & 0xFF;
  //
  //  buff = gyr.z * j;
  //  //buff = 1 * -j;
  //    bytes[25] = (buff >> 24) & 0xFF;
  //    bytes[26] = (buff >> 16) & 0xFF;
  //    bytes[27] = (buff >> 8) & 0xFF;
  //    bytes[28] = (buff) & 0xFF;
  //
  //  buff = magValue.x * j;
  // //buff = 1 * -j;
  //    bytes[29] = (buff >> 24) & 0xFF;
  //    bytes[30] = (buff >> 16) & 0xFF;
  //    bytes[31] = (buff >> 8) & 0xFF;
  //    bytes[32] = (buff) & 0xFF;
  //
  //  buff = magValue.y * j;
  //  //buff = 1 * -j;
  //    bytes[33] = (buff >> 24) & 0xFF;
  //    bytes[34] = (buff >> 16) & 0xFF;
  //    bytes[35] = (buff >> 8) & 0xFF;
  //    bytes[36] = (buff) & 0xFF;
  //
  //  buff = magValue.z * j;
  //  //buff = 1 * -j;
  //  bytes[37] = (buff >> 24) & 0xFF;
  //  bytes[38] = (buff >> 16) & 0xFF;
  //  bytes[39] = (buff >> 8) & 0xFF;
  //  bytes[40] = (buff) & 0xFF;

  //  Serial.print(sizeof(bytes));
    Serial.print(str);
    Serial.print("\n");
 
//Serial.print(str);
//Serial.print("\n");

}
