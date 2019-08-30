// Kalman Filter v4
// add: use serial to communicate 
// streaming IMU value to Matlab via serial and WiFi

// Library included
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Wire.h>

#define MPU9250_ADDRESS 0x68
// the register address of acc_x higher byte
#define ACC_XOUT_H 59

//Global variables
unsigned long t;

// Declare function
void I2Cread ();
void I2CwriteByte ();
void printWiFiStatus();
void PrintValue();  //leave for debug
void cut4byte (uint32_t, uint8_t*);

// WiFi Information
char ssid[] = "thermal";
char pass[] = "h0t5tuff";

// Remote Information
int remote_Port = 9001;
uint8_t remote_IP[4] = {192, 168, 1, 36};

// Local Information
int local_Port = 8888;

// contruct UDP class
WiFiUDP Udp;

void setup()
{
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

  // Serial Setup
  Serial.begin(9600);
//  while (!Serial);

  // WiFi-chip setup
  WiFi.setPins(8, 7, 4, 2);
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.print("No shield detected, program terminate");
    while (true);
  }

  int Status = WL_IDLE_STATUS;
  while (Status != WL_CONNECTED)
  {
    Serial.println("connecting...");
    Status = WiFi.begin(ssid, pass);
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
  /////////////IMU_reading_start/////////////////////
  uint8_t temp[14];
  I2Cread(MPU9250_ADDRESS, ACC_XOUT_H, 14, temp);

  // Time stamp, and into 4 discrete bytes
  t = millis();
  uint8_t Time[4];
  cut4byte(t, Time);

  //do invert and include the time stamp
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

  //create array "raw" to store raw value
  int16_t raw[7];
  for (int i = 0; i < 7; i++)
  {
    int index = i * 2;
    raw[i] = (temp[index] << 8 | temp[index + 1]);
  }

  //create array "measure" to store uncalibrated value
  float measure[8];

  //Accerometer Data
  //250 dps into 2^15 resolution, we need to transform it back to deg per sec
  for (int i = 0; i < 3; i++)
    measure[i] = raw[i] / 16384.0;

  //Temperature data
  measure[3] = raw[3] / 328.0 + 21;

  //Gyro Data
  // 2G into 2^15 resolution, we need to transform it back to G per sec
  for (int i = 4; i < 7; i++)
    measure[i] = raw[i] / 131.0;
  measure [7] = t; 

  /////////////IMU_reading_end///////////////////////

  /////////////Serial_sending_start//////////////////
  for (int i=0; i<18; i++) 
  {
    Serial.write(Temp[i]);
  }
  /////////////Serial_sending_end////////////////////
  
  /////////////UDP_connect_start/////////////////////

  //connect to the remote port
  if (!Udp.beginPacket(remote_IP, remote_Port))
  {
    Serial.println("fail to connect to the host, program terminate");
    while (true);
  }

  //Send the data
  Udp.write(Temp, 18);

//  PrintValue (Temp, raw, measure);
  //End transmission
  Udp.endPacket();
  delay(50);

  /////////////UDP_connect_end/////////////////////
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

void printWiFiStatus() {
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

void PrintValue (uint8_t* Temp, int16_t* raw, float* measure)
{
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // unknitted
  Serial.println("unknitted:");
  for (int i = 0; i < 18; i++)
  {
    Serial.print(i + 1); Serial.print("\t");
  } Serial.println();
  for (int i = 0; i < 18; i++)
  {
    Serial.print(Temp[i]); Serial.print("\t");
  } Serial.println();
  // raw
  Serial.println("raw:");
  for (int i = 0; i < 7; i++)
  {
    Serial.print(i + 1); Serial.print("\t");
  } Serial.println();
  for (int i = 0; i < 7; i++)
  {
    Serial.print(raw[i]); Serial.print("\t");
  } Serial.println();
  // Measure
  String Name = "ax\t ay\t az\t T\t gx\t gy\t gz\t time";
  Serial.println("readable:");
  Serial.println(Name);
  for (int i = 0; i < 8; i++)
  {
    Serial.print(measure[i]); Serial.print("\t");
  } Serial.println();

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
