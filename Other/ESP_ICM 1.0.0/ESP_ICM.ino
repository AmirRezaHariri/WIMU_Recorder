#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
int status;
#define SendKey 0  //Button to send data Flash BTN on NodeMCU

int port = 8888;  //Port number1
WiFiServer server(port);
//Server connect to WiFi Network
const char *ssid = "FUM_WIMU";  //Enter your wifi SSID
const char *password = "12345xyz";  //Enter your wifi Password
int a = 0, count = 0, sample_time = 10;
//int down,middle,up;
long count1 = 0;
byte bytes[41];
float t1 = 0, t2 = 0;

void ICM_read(void);
//=======================================================================
//                    Power on setup
//=======================================================================
void setup()
{
  pinMode(D3, INPUT_PULLUP);   //down
  pinMode(D4, INPUT_PULLUP);   //middle
  pinMode(D8, INPUT_PULLUP);  //Up  

  //pinMode(10, OUTPUT);
  //digitalWrite(10, HIGH);
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(SendKey, INPUT_PULLUP); //Btn to send data
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //Connect to wifi

  // Wait for connection
  Serial.println("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  server.begin();
  Serial.print("Open Telnet and connect to IP:");
  Serial.print(WiFi.localIP());
  Serial.print(" on port ");
  Serial.println(port);
  Serial.print("sample time (ms): ");
  Serial.println(sample_time);

  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  }
  else {
    Serial.println("ICM20948 is connected");
  }

  if (!myIMU.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  }
  else {
    Serial.println("Magnetometer is connected");
  }

  /* Check ICM20948 */
  // byte reg = myIMU.whoAmI();
  //Serial.println(reg);

  /******************* Basic Settings ******************/

  /*  This is a method to calibrate. You have to determine the minimum and maximum
      raw acceleration values of the axes determined in the range +/- 2 g.
      You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
      The parameters are floats.
      The calibration changes the slope / ratio of raw acceleration vs g. The zero point is
      set as (min + max)/2.
  */
  //myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16520.0, 16690.0);

  /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z.
     The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its
     x,y-plane. The more you deviate from this, the less accurate will be your results.
     It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
     The function also measures the offset of the gyroscope data. The gyroscope offset does not
     depend on the positioning.
     This function needs to be called after setAccOffsets but before other settings since it will
     overwrite settings!
  */
  //  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  //  delay(1000);
  //  myIMU.autoOffsets();
  //  Serial.println("Done!");

  /*  The gyroscope data is not zero, even if you don't move the ICM20948.
      To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
      using the +/- 250 degrees/s range.
      Use either autoOffset or setGyrOffsets, not both.
  */
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);

  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
      ICM20948_ACC_RANGE_4G      4 g
      ICM20948_ACC_RANGE_8G      8 g
      ICM20948_ACC_RANGE_16G    16 g
  */
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);

  /*  Choose a level for the Digital Low Pass Filter or switch it off.
      ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF

      DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
        0              246.0               1125/(1+ASRD)
        1              246.0               1125/(1+ASRD)
        2              111.4               1125/(1+ASRD)
        3               50.4               1125/(1+ASRD)
        4               23.9               1125/(1+ASRD)
        5               11.5               1125/(1+ASRD)
        6                5.7               1125/(1+ASRD)
        7              473.0               1125/(1+ASRD)
        OFF           1209.0               4500

        ASRD = Accelerometer Sample Rate Divider (0...4095)
        You achieve lowest noise using level 6
  */
  myIMU.setAccDLPF(ICM20948_DLPF_6);

  /*  Acceleration sample rate divider divides the output rate of the accelerometer.
      Sample rate = Basic sample rate / (1 + divider)
      It can only be applied if the corresponding DLPF is not off!
      Divider is a number 0...4095 (different range compared to gyroscope)
      If sample rates are set for the accelerometer and the gyroscope, the gyroscope
      sample rate has priority.
  */
  //myIMU.setAccSampleRateDivider(10);

  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
      ICM20948_GYRO_RANGE_500       500 degrees per second
      ICM20948_GYRO_RANGE_1000     1000 degrees per second
      ICM20948_GYRO_RANGE_2000     2000 degrees per second
  */
  //myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);

  /*  Choose a level for the Digital Low Pass Filter or switch it off.
      ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF

      DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
        0              196.6               1125/(1+GSRD)
        1              151.8               1125/(1+GSRD)
        2              119.5               1125/(1+GSRD)
        3               51.2               1125/(1+GSRD)
        4               23.9               1125/(1+GSRD)
        5               11.6               1125/(1+GSRD)
        6                5.7               1125/(1+GSRD)
        7              361.4               1125/(1+GSRD)
        OFF          12106.0               9000

        GSRD = Gyroscope Sample Rate Divider (0...255)
        You achieve lowest noise using level 6
  */
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  /*  Gyroscope sample rate divider divides the output rate of the gyroscope.
      Sample rate = Basic sample rate / (1 + divider)
      It can only be applied if the corresponding DLPF is not OFF!
      Divider is a number 0...255
      If sample rates are set for the accelerometer and the gyroscope, the gyroscope
      sample rate has priority.
  */
  //myIMU.setGyrSampleRateDivider(10);

  /*  Choose a level for the Digital Low Pass Filter.
      ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF

      DLPF          Bandwidth [Hz]      Output Rate [Hz]
        0             7932.0                    9
        1              217.9                 1125
        2              123.5                 1125
        3               65.9                 1125
        4               34.1                 1125
        5               17.3                 1125
        6                8.8                 1125
        7             7932.0                    9


        GSRD = Gyroscope Sample Rate Divider (0...255)
        You achieve lowest noise using level 6
  */
  myIMU.setTempDLPF(ICM20948_DLPF_6);

  /* You can set the following modes for the magnetometer:
     AK09916_PWR_DOWN          Power down to save energy
     AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by
                               calling setMagOpMode(AK09916_TRIGGER_MODE)
     AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
     AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
     AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
     AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
  */
  myIMU.setMagOpMode(AK09916_CONT_MODE_50HZ);
}

//=======================================================================
//                    Loop
//=======================================================================

void loop()
{
  WiFiClient client = server.available();
  if (client) {
    if (client.connected())
    {
      Serial.println("Client Connected");
    }
    while (client.connected()) {
      t1 = millis();
      ICM_read();
      //Send Data to connected client
      client.write(bytes,41);
      t2 = millis();
//      Serial.println(count1);
      //Serial.println((t2-t1));
      if ((t2 - t1) < sample_time)
        delay(sample_time - (t2 - t1));
      else
        continue;
    }
    client.stop();
    Serial.println("Client disconnected");
    count1 = 0;
  }
}

void ICM_read() {

  myIMU.readSensor();
  xyzFloat gValue = myIMU.getGValues();
  xyzFloat gyr = myIMU.getGyrValues();
  xyzFloat magValue = myIMU.getMagValues();
  float temp = myIMU.getTemperature();
  float resultantG = myIMU.getResultantG(gValue);

  //down= digitalRead(13);
  //middle= digitalRead(14);
  //up= digitalRead(12);

  count1 = count1 + sample_time;
  
  long int buff;
  long int reg = 5000;

  buff = int((count1 + reg) * 10000);
  bytes[0] = (buff >> 24) & 0xFF;
  bytes[1] = (buff >> 16) & 0xFF;
  bytes[2] = (buff >> 8) & 0xFF;
  bytes[3] = (buff) & 0xFF;
  
  buff = int((gValue.x + reg) * 10000);
  bytes[4] = (buff >> 24) & 0xFF;
  bytes[5] = (buff >> 16) & 0xFF;
  bytes[6] = (buff >> 8) & 0xFF;
  bytes[7] = (buff) & 0xFF;
  
  buff = int((gValue.y + reg) * 10000);
  bytes[8] = (buff >> 24) & 0xFF;
  bytes[9] = (buff >> 16) & 0xFF;
  bytes[10] = (buff >> 8) & 0xFF;
  bytes[11] = (buff) & 0xFF;

  buff = int((gValue.z + reg) * 10000);
  bytes[12] = (buff >> 24) & 0xFF;
  bytes[13] = (buff >> 16) & 0xFF;
  bytes[14] = (buff >> 8) & 0xFF;
  bytes[15] = (buff) & 0xFF;
  
  buff = int((gyr.x + reg) * 10000);
  bytes[16] = (buff >> 24) & 0xFF;
  bytes[17] = (buff >> 16) & 0xFF;
  bytes[18] = (buff >> 8) & 0xFF;
  bytes[19] = (buff) & 0xFF;
  
  buff = int((gyr.y + reg) * 10000);
  bytes[20] = (buff >> 24) & 0xFF;
  bytes[21] = (buff >> 16) & 0xFF;
  bytes[22] = (buff >> 8) & 0xFF;
  bytes[23] = (buff) & 0xFF;
  
  buff = int((gyr.z + reg) * 10000);
  bytes[24] = (buff >> 24) & 0xFF;
  bytes[25] = (buff >> 16) & 0xFF;
  bytes[26] = (buff >> 8) & 0xFF;
  bytes[27] = (buff) & 0xFF;
  
  buff = int((magValue.x + reg) * 10000);
  bytes[28] = (buff >> 24) & 0xFF;
  bytes[29] = (buff >> 16) & 0xFF;
  bytes[30] = (buff >> 8) & 0xFF;
  bytes[31] = (buff) & 0xFF;
  
  buff = int((magValue.y + reg) * 10000);
  bytes[32] = (buff >> 24) & 0xFF;
  bytes[33] = (buff >> 16) & 0xFF;
  bytes[34] = (buff >> 8) & 0xFF;
  bytes[35] = (buff) & 0xFF;
  
  buff = int((magValue.z + reg) * 10000);
  bytes[36] = (buff >> 24) & 0xFF;
  bytes[37] = (buff >> 16) & 0xFF;
  bytes[38] = (buff >> 8) & 0xFF;
  bytes[39] = (buff) & 0xFF;

  
  byte buff2 = 0b10000000;
  bool down= digitalRead(D3);
  bool middle= digitalRead(D4);
  bool up= digitalRead(D8);
  bitWrite(buff2, 0, !down);
  bitWrite(buff2, 1, !middle);
  bitWrite(buff2, 2, up);
  bytes[40] = buff2;
//  Serial.print(buff2, BIN);
//  Serial.print('\n');

//  char str[1000];
//  sprintf(str,"%i,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f;",count1, \
//  gValue.x, gValue.y, gValue.z, gyr.x, gyr.y, gyr.z, magValue.x, magValue.y, magValue.z);
//  Serial.print(str);
}
