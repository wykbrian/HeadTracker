//sending IMU value to JUCE and Matlab
//JUCE: OSC via UDP
//Matlab: serial 

// Library included
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>

//I2C setup for IMU 
#define MPU9250_ADDRESS 0x68
// the register address of acc_x higher byte
#define ACC_XOUT_H 59

// Declare function
void printWiFiStatus();
void I2Cread ();
void I2CwriteByte ();
void cut4byte (uint32_t, uint8_t*);

// WiFi Information
char ssid[] = "thermal";
char pass[] = "h0t5tuff";

// Remote Information (host: Mac)
int remote_Port = 9001;
uint8_t remote_IP[4] = {192, 168, 1, 32};
const char* OSC_address = "/juce/0" ;

// Local Information
int local_Port = 8888;

// contruct UDP object
WiFiUDP Udp;

//Global variables
unsigned long t;

void setup()
{
  // Serial Setup
  Serial.begin(9600);
  //while (!Serial);

  //I2C Set-up 
  Wire.begin();
  // set low pass filter for gyro
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
  // set low pass filter for acclerometer
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // configure range for gyro (250 dps)
  I2CwriteByte(MPU9250_ADDRESS, 27, 0x00);
  // configure range for acclerometer (+-2G)
  I2CwriteByte(MPU9250_ADDRESS, 28, 0x00);
  // set the clock reference into auto-mode
  I2CwriteByte(MPU9250_ADDRESS, 107, 0x01);

  // WiFi-chip setup
  WiFi.setPins(8, 7, 4, 2);
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.print("No shield detected, program terminate");
    while (true);
  }

  int Status = WiFi.begin(ssid, pass);
  // why not WiFi.status() != WL_CONNECTED ? rather than int WiFi.begin
  // 6 = WL_IDLE_STATUS; 3 = WL_CONNECTED
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("connecting...");
    // try another method to connect to the WiFi 
    Status = WiFi.begin(ssid, pass);
    Serial.println(Status);
    delay(10000);
  }
  Serial.println("connected to WiFi");
  printWiFiStatus();

  // Initialize the UDP library and listen at local port
  if (!Udp.begin(local_Port))
  {
    Serial.println("fail to connect to the server, program terminate");
    while (true);
  }

}

void loop()
{
  //read time 
  t  = millis();
  float T = t;

  //cut into 4 discrete bytes for serial transport
  uint8_t Time[4];
  cut4byte(t, Time);
  
  /////////////IMU_reading_start/////////////////////
  uint8_t temp[14];  
  I2Cread(MPU9250_ADDRESS, ACC_XOUT_H, 14, temp);  

  //do invert and include the time stamp (Matlab)
  uint8_t Temp[18];
  for (int i = 0; i < 7; i++)
  {
    int index = i * 2;
    Temp[index] = temp[index + 1];
    Temp[index + 1] = temp[index];
  }
  Temp [14] = Time[0];
  Temp [15] = Time[1];
  Temp [16] = Time[2];
  Temp [17] = Time[3];

  //JUCE
  //Accerometer Data
  int16_t ax_raw = (temp[0] << 8 | temp[1]);
  // 250 dps into 2^15 resolution, we need to transform it back to deg per sec
  float ax = ax_raw / 16384.0;
  int16_t ay_raw = (temp[2] << 8 | temp[3]);
  float ay = ay_raw / 16384.0;
  int16_t az_raw = (temp[4] << 8 | temp[5]);
  float az = az_raw / 16384.0;
  
  //Temperature data
  int16_t tp_raw = (temp[6] << 8 | temp[7]);
  float tp = tp_raw / 328.0 + 21;

  //Gyro Data
  int16_t gx_raw = (temp[8] << 8 | temp[9]);
  // 2G into 2^15 resolution, we need to transform it back to G per sec
  float gx = gx_raw / 131.0;
  int16_t gy_raw = (temp[10] << 8 | temp[11]);
  float gy = gy_raw / 131.0;
  int16_t gz_raw = (temp[12] << 8 | temp[13]);
  float gz = gz_raw / 131.0;

  /////////////IMU_reading_end///////////////////////

  /////////////Serial_sending_start//////////////////
  for (int i=0; i<18; i++) 
  {
    Serial.write(Temp[i]);
  }

  /////////////Serial_sending_end////////////////////

  /////////////UDP_sending_start//////////////////
  
  if (!Udp.beginPacket(remote_IP, remote_Port))
  {
    Serial.println("fail to connect to the host, program terminate");
    while (true);
  }

  //Construct OSC objcet and set OSC_Address
  OSCMessage msg(OSC_address);

  //transfer IMU value into oscMessage
  msg.add(ax).add(ay).add(az).add(gx).add(gy).add(gz).add(T);

  //Send the OSC message with UDP
  msg.send(Udp);

  //End transmission
  Udp.endPacket();

  //clear the space
  msg.empty();
  
  /////////////UDP_sending_end//////////////////

  delay(50);

}

void printWiFiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void I2Cread (uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  // why not use false
  Wire.endTransmission();
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
  {
    Data[index++] = Wire.read();
  }
}

void I2CwriteByte (uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void cut4byte (uint32_t t, uint8_t* Array)
{
  uint32_t temp;
  uint32_t bits[4] = {255, 65280, 16711680, 4278190080};
  for (int i = 0; i < 4; i++)
  {
    temp = (t & bits[i]) >> (8 * i);
    Array[i] = temp & 255;
  }
}
