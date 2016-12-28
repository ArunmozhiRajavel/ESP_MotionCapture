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

#define BUFFER_SIZE 100
#define INTERRUPT_PIN 14
#define USE_SERIAL Serial

const int MPU_addr = 0x68;                          // I2C address of the MPU-6050
const char* mqtt_server = "m13.cloudmqtt.com";      // cloudmqtt server
const int mqtt_port = 17673;                        // cloudmqtt port
const char *mqtt_user = "crfqbqvi";                 // cloudmqtt username
const char *mqtt_pass = "_9noZYPyS7D9";             // cloudmqtt password

bool updateFirmware = 0;                            // status flag for new firmware update

// MPU control/status vars
bool dmpReady = false;                  // set true if DMP init was successful
uint8_t mpuIntStatus;                   // holds actual interrupt status byte from MPU
uint8_t devStatus;                      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                 // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

Quaternion q;                                           // quaternion container
ESP8266WiFiMulti WiFiMulti;                             // ESP8266WiFiMulti object to connect to any of the given list of WiFi networks
HTTPClient http;
WiFiClient wclient;                                     // WiFiClient object to be used in the PubSubClient
PubSubClient client(wclient, mqtt_server,  mqtt_port);  // client object to connect to mqtt broker
MPU6050 mpu;

void dmpDataReady()
{
  mpuInterrupt = true;
}

// Function to be executed on receiving a message from the subscribed topic
// Specifically used to set and clear firmware update flag based on the values in the subscribed topic("firmwareupdate")
void callback(const MQTT::Publish& pub)
{
  Serial.print(pub.topic());
  Serial.print(" => ");
  Serial.println(pub.payload_string());
  if (pub.has_stream())
  {
    uint8_t buf[BUFFER_SIZE];
    int read;
    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE))
    {
      Serial.write(buf, read);
    }
    pub.payload_stream()->stop();

    if (buf[0] == 1)
    {
      updateFirmware = 1;
    }
    Serial.println("");
  }
  else
  {
    Serial.println("No Stream");
    if (pub.payload_string() == "1")
    {
      updateFirmware = 1;                                       // Sets the updateFirmware flag so that in the next loop firmware update begins
    }
  }
}

void setup() {


  pinMode(LED_BUILTIN, OUTPUT);     //

  Serial.begin(115200);              Serial.setDebugOutput(true);

  //Begin I2C data transfer
  Wire.begin(4, 5);                 // SCL <==> D1 , SDA <==> D2
  Wire.setClock(400000);            // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);                 // PWR_MGMT_1 register
  Wire.write(0);                    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Initialize DMP
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(500);

  // Gyro offsets.As the sensor bby default will have some offset value
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0)
  {
    // turn on the DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set DMP Ready flag so the main loop() function can use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Needed to avoid boot crashing.Should investigate further.Since its one time delay during setup won't affect the performance of the loop
  for (uint8_t t = 4; t > 0; t--)
  {
    Serial.printf("WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  //ssid and password for the list of networks that will be available and wait till it gets connected
  WiFiMulti.addAP("InstaOffice", "nov16#offices@insta");
  WiFiMulti.addAP("Arunmozhi", "q1w2e3r4");
  while (WiFiMulti.run() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
}

// Contains 2 parts
// 1.Checks for firmware update.If available updates the firmware and clears the updateFirmware flag
// 2.Calls the main program where the actual functionalities are available
void loop()
{
  if ((WiFiMulti.run() == WL_CONNECTED))
  {
    if (!client.connected())
    {
      Serial.println("Connecting to MQTT server");
      //Authenticating the client object
      if (client.connect(MQTT::Connect("mqtt_client_name").set_auth(mqtt_user, mqtt_pass)))
      {
        Serial.println("Connected to MQTT server");
        //Subscribe to topic "firmwareupdate"
        client.set_callback(callback);
        client.subscribe("firmwareupdate");
      }
      else
      {
        Serial.println("Could not connect to MQTT server");
      }
    }
    else
    {
      client.loop();
      if (updateFirmware) {
        Serial.println("Updating firware");
        digitalWrite(LED_BUILTIN, LOW);
        t_httpUpdate_return ret = ESPhttpUpdate.update("http://s3.ap-south-1.amazonaws.com/firmwaretest1/httpUpdate.ino.bin", "v1");
        switch (ret) {
          case HTTP_UPDATE_FAILED:
            Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;

          case HTTP_UPDATE_NO_UPDATES:
            Serial.println("HTTP_UPDATE_NO_UPDATES");
            break;

          case HTTP_UPDATE_OK:
            Serial.println("HTTP_UPDATE_OK");
            client.publish("firmwareupdate", "2");
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            updateFirmware = 0;
            ESP.restart();
            break;
        }
      }
      else
      {
        //Serial.println("No update found");
      }

      //Call the actual program
      program();
    }
  }
  else
  {
    Serial.println("Unable to connect wifi");
  }
}

void program()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (indicates code efficiency should be improved to handle the overflow)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset FIFO buffer
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length
    while (fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    //String object to form a concatenated data to be published in the topic "sensorData"
    String data = "";
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    data = data + String(q.w, 4) + ";" + String(q.x, 4) + ";" + String(q.y, 4) + ";" + String(q.z, 4);
    Serial.println(data);
    client.publish("sensorData", data);
  }
  else
  {
    Serial.println("Skipped");
  }
}

