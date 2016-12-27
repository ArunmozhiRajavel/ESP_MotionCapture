#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <stdlib.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include<Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define USE_SERIAL Serial
#define BUFFER_SIZE 100
#define INTERRUPT_PIN 14

const int MPU_addr = 0x68; // I2C address of the MPU-6050
const long interval = 10000;
const char *ssid = "InstaOffice"; 
const char *pass = "nov16#offices@insta"; 
const char* mqtt_server = "m13.cloudmqtt.com";
const int mqtt_port = 17673;
const char *mqtt_user = "crfqbqvi";
const char *mqtt_pass = "_9noZYPyS7D9";
bool updateFirmware = 0;
unsigned long previousMillis = 0;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//float q[4];             // [w, x, y, z]         quaternion container
//char qw[10];
//char qx[10];
//char qy[10];
//char qz[10];

char data[14][10];

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

ESP8266WiFiMulti WiFiMulti;
HTTPClient http;
WiFiClient wclient;  //Declares a WifiClient Object using ESP8266WiFi
PubSubClient client(wclient, mqtt_server,  mqtt_port);  //instanciates client object
MPU6050 mpu;

void callback(const MQTT::Publish& pub) {
  USE_SERIAL.print(pub.topic());
  USE_SERIAL.print(" => ");
  USE_SERIAL.println(pub.payload_string());
  if (pub.has_stream()) {
    uint8_t buf[BUFFER_SIZE];
    int read;
    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE)) {
      Serial.write(buf, read);
    }
    pub.payload_stream()->stop();

    //Check if the buffer is -1
    if (buf[0] == 1) {
      updateFirmware = 1;
    }
    USE_SERIAL.println("");
  } else
  {
    USE_SERIAL.println("No Stream");
    if (pub.payload_string() == "1")
    {
      updateFirmware = 1;
    }
  }
}

void setup() {
  Wire.begin(4, 5);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  pinMode(LED_BUILTIN, OUTPUT);
  USE_SERIAL.begin(115200);
  //USE_SERIAL.setDebugOutput(true);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(500);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  WiFiMulti.addAP("InstaOffice", "nov16#offices@insta");
  WiFiMulti.addAP("Arunmozhi", "q1w2e3r4");

  while (WiFiMulti.run() != WL_CONNECTED) {
    USE_SERIAL.print(".");
    delay(500);
  }
  //WiFiMulti.addAP("InstaOffice", "nov16#offices@inst");
}

void loop() {
  //USE_SERIAL.println(millis());
  if ((WiFiMulti.run() == WL_CONNECTED))
  {
    if (!client.connected()) {
      USE_SERIAL.println("Connecting to MQTT server");
      //Authenticating the client object
      if (client.connect(MQTT::Connect("mqtt_client_name")
                         .set_auth(mqtt_user, mqtt_pass))) {
        USE_SERIAL.println("Connected to MQTT server");
        //Subscribe code
        client.set_callback(callback);
        client.subscribe("firmwareupdate");
      } else {
        USE_SERIAL.println("Could not connect to MQTT server");
      }
    }
    else  {
      client.loop();
      if (updateFirmware) {
        USE_SERIAL.println("Updating firware");
        t_httpUpdate_return ret = ESPhttpUpdate.update("http://s3.ap-south-1.amazonaws.com/firmwaretest1/httpUpdate.ino.bin", "v1");
        switch (ret) {
          case HTTP_UPDATE_FAILED:
            USE_SERIAL.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;

          case HTTP_UPDATE_NO_UPDATES:
            USE_SERIAL.println("HTTP_UPDATE_NO_UPDATES");
            break;

          case HTTP_UPDATE_OK:
            USE_SERIAL.println("HTTP_UPDATE_OK");
            client.publish("firmwareupdate", "2");
            delay(50);
            updateFirmware = 0;
            ESP.restart();
            break;
        }
      }
      else
      {
        //USE_SERIAL.println("No update found");
      }
      program();
    }
  }
  else
  {
    USE_SERIAL.println("Unable to connect wifi");
  }
}

void program()
{

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display quaternion values in InvenSense Teapot demo format:
    //    teapotPacket[2] = fifoBuffer[0];
    //    teapotPacket[3] = fifoBuffer[1];
    //    teapotPacket[4] = fifoBuffer[4];
    //    teapotPacket[5] = fifoBuffer[5];
    //    teapotPacket[6] = fifoBuffer[8];
    //    teapotPacket[7] = fifoBuffer[9];
    //    teapotPacket[8] = fifoBuffer[12];
    //    teapotPacket[9] = fifoBuffer[13];
    //    Serial.write(teapotPacket, 14);
    //    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    Serial.println(millis());
    String data = "";
    data = data + fifoBuffer[0] + ";" + fifoBuffer[1] + ";" + fifoBuffer[4] + ";" + fifoBuffer[5] + ";" + fifoBuffer[8] + ";" + fifoBuffer[9] + ";" + fifoBuffer[12] + ";" + fifoBuffer[13];
    //mpu.dmpGetQuaternion(&q, fifoBuffer);

    //    dtostrf(q.w, 6, 4, qw);
    //    dtostrf(q.x, 6, 4, qx);
    //    dtostrf(q.y, 6, 4, qy);
    //    dtostrf(q.z, 6, 4, qz);
    //    String data = "";
    //    data = data + qw + ";" + qx + ";" + qy + ";" + qz;
    client.publish("check", data);
    Serial.println(millis());
    //Serial.write(teapotPacket, 14);
    //teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
  }
}


