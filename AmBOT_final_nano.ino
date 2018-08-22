#define LOG_INPUT false
#define LOG_OUTPUT false
#define LOG_WIFI false
#define PID_LOG false

#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <ctype.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>

#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <WiFiUdp.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 246.5;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 54;
double Kd = 2;
double Ki = 100;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;

// PINOUTS

// mcu6050 (gravity): 
// VCC
// GND
// D1
// D2
// nc
// nc
// nc
// D8

int INTERRUPT_PIN = 15;

//MOTOR CONTROLLER
int ENA = 2;  //D4
int IN1 = 14; //D5
int IN2 = 12; //D6 
int IN3 = 13; //D7
int IN4 = 16; //D0
int ENB = 3;  //RX

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//timers
long time1Hz = 0;
long time5Hz = 0;

char blynk_token[34] = "blynk tokrn here";
bool shouldSaveConfig = false;

bool operating = false;
double leftShift = 0.0;
double rightShift = 0.0;
int minAbsSpeed = 20;

ESP8266WebServer server(80); //creating the server at port 80

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// Multicast declarations
IPAddress ipBroadcast(255, 255, 255, 255);
unsigned int portBroadcast = 61565; // port to broadcast on must be between1024...65000
bool gotFirstCommand = false;
unsigned long lastTimeIpSent = 0;
char ipAddressBuff[40];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
  operating = false;
  
  Serial.begin(115200);
  setupWifi();
  
  setupBRObot();

}

void setupBRObot()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
//        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID

        analogWriteRange(255);
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
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
}

void setupWifi() {
  char boot_info_string[100];
  sprintf(boot_info_string, "BtMd: %u rsn: %s", ESP.getBootMode(), ESP.getResetReason().c_str());
  Serial.print("Boot: ");
  Serial.println(boot_info_string);
//clean FS, for testing
//  SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(blynk_token, json["blynk_token"]);

          Serial.println("\ncopied vars");
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
//  WiFiManager wifiManager// = createWifiManager();
//


  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

  WiFiManagerParameter custom_blynk_token("blynk_token", "blynk token", blynk_token, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
//  wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_blynk_token);

//  //set config save notify callback
//  wifiManager.setSaveConfigCallback(saveConfigCallback);
//
//  //set static ip
////  wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
//  
//  //add all your parameters here
//  wifiManager.addParameter(&custom_mqtt_server);
//  wifiManager.addParameter(&custom_mqtt_port);
//  wifiManager.addParameter(&custom_blynk_token);
//  wifiManager.addParameter(&custom_topic);

  //reset settings - for testing
//  wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);
  boolean needDropSettings = false;
  if (needDropSettings){
    Serial.println("Drop wifi settings");
    wifiManager.resetSettings();
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
 
  }

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoRelayConfig", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  IPAddress ip = WiFi.localIP();
  sprintf(ipAddressBuff, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

  Serial.println("local ip");
  Serial.println(ip);

  Serial.print("blynk_token: ");
  Serial.println(blynk_token);

  Serial.println("Server setup");
  server.on("/ctrl", ctrlHandleFunc);
  server.on("/", ctrlHandleFunc);
    
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("  --= SETUP DONE =--");
}


//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void sendIpAddressViaUDP() {
  //Udp.begin(portBroadcast);
  
  Udp.beginPacket(ipBroadcast, portBroadcast);
  Udp.write("mrBRObot:IP=");
  Udp.write(ipAddressBuff);
  Udp.endPacket(); 
  
  //Udp.stop();
}


void ctrlHandleFunc(){
   #if LOG_WIFI
   Serial.println("GOT request:");
   #endif

   gotFirstCommand = true;
   if (server.args() > 0){
    for (int i = 0; i < server.args();i++){

        String key = server.argName(i);
        String val = server.arg(i);

        #if LOG_WIFI
        Serial.print("Name: "); Serial.println(key);
        Serial.print("Value: "); Serial.println(val);
        #endif
        
        handleParam(key, val);
      }
   }
   server.send(200, "text/html", "OK");
   #if LOG_WIFI
   Serial.println("END request");
   #endif
}

String KP = "kp";
String KI = "ki";
String KD = "kd";
String LEFT_SHIFT = "lshift";
String RIGHT_SHIFT = "rshift";
String TARGET = "target";
String LEFT_COEF = "lcoef";
String RIGHT_COEF = "rcoef";
String MINSTEP = "minstep";
String SAMPLETIME = "sampletime";
String OPERATE = "operate";

void handleParam(String key, String val){
  val.trim();
  
  if (val.length() == 0) {
    #if LOG_WIFI    
    Serial.print(key); Serial.print(" -> "); Serial.println("DROPPED EMPTY"); 
    #endif
    return;
  }
    
  if (KP.equalsIgnoreCase(key)){
    Kp = val.toFloat();
    pid.SetTunings(Kp, Ki, Kd);
    #if LOG_WIFI
    Serial.print("Kp -> "); Serial.println(Kp); 
    #endif
  } else if (KI.equalsIgnoreCase(key)){
    Ki = val.toFloat();
    pid.SetTunings(Kp, Ki, Kd);
    #if LOG_WIFI
    Serial.print("Ki -> "); Serial.println(Ki); 
    #endif
  } else if (KD.equalsIgnoreCase(key)){
    Kd = val.toFloat();
    pid.SetTunings(Kp, Ki, Kd);
    #if LOG_WIFI
    Serial.print("Kd -> "); Serial.println(Kd); 
    #endif
  } else if (SAMPLETIME.equalsIgnoreCase(key)){
    int stime = val.toInt();
    if (stime >= 1){
       pid.SetSampleTime(stime);
    }
    #if LOG_WIFI
    Serial.print("stime -> "); Serial.println(stime); 
    #endif
  } else if (LEFT_SHIFT.equalsIgnoreCase(key)){
    leftShift = val.toFloat();
    #if LOG_WIFI
    Serial.print("leftShift -> "); Serial.println(leftShift); 
    #endif
  } else if (RIGHT_SHIFT.equalsIgnoreCase(key)){
    rightShift = val.toFloat();
    #if LOG_WIFI
    Serial.print("rightShift -> "); Serial.println(rightShift); 
    #endif
  } else if (TARGET.equalsIgnoreCase(key)){
    setpoint = val.toFloat();
    #if LOG_WIFI
    Serial.print("originalSetpoint -> "); Serial.println(originalSetpoint); 
    #endif
  } else if (RIGHT_COEF.equalsIgnoreCase(key)){
    motorSpeedFactorRight = val.toFloat();
    #if LOG_WIFI
    Serial.print("motorSpeedFactorRight -> "); Serial.println(motorSpeedFactorRight); 
    #endif
  } else if (LEFT_COEF.equalsIgnoreCase(key)){
    motorSpeedFactorLeft = val.toFloat();
    #if LOG_WIFI
    Serial.print("motorSpeedFactorLeft -> "); Serial.println(motorSpeedFactorLeft); 
    #endif
  } else if (MINSTEP.equalsIgnoreCase(key)){
    minAbsSpeed = val.toInt();
    #if LOG_WIFI
    Serial.print("minAbsSpeed -> "); Serial.println(minAbsSpeed); 
    #endif
  } else if (OPERATE.equalsIgnoreCase(key)){
    operating = val.toInt() != 0;
    #if LOG_WIFI
    Serial.print("operating -> "); Serial.println(operating); 
    #endif
  } else {
    #if LOG_WIFI
    Serial.print(key); Serial.print(" -> "); Serial.println("IGNORED"); 
    #endif
  }
}

void loop()
{
     server.handleClient(); //this is required for handling the incoming requests

    if (!gotFirstCommand){
      unsigned long now = millis();
            
      if (now >= lastTimeIpSent + 2000){      
        Serial.println("Sending IP addres via UDP"); 
        lastTimeIpSent = now;
        sendIpAddressViaUDP();
      }
    }
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        //no mpu data - performing PID calculations and output to motors
        pid.Compute();
        double lSpeed = output + leftShift;
        double rSpeed = output + rightShift;
        #if LOG_OUTPUT
        Serial.print(F("output "));
        Serial.print(output);
        Serial.print(" lshift: ");
        Serial.print(leftShift);
        Serial.print(" lshift: ");
        Serial.print(rightShift);
        Serial.print(" l: ");
        Serial.print(lSpeed);
        Serial.print(" r: ");
        Serial.println(rSpeed);
        #endif
        if (operating) {
          
          motorController.move(lSpeed, rSpeed, minAbsSpeed);
        } else {
          motorController.move(0, 0, minAbsSpeed);
        }
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180/M_PI + 180;

        #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t");
            Serial.println(input);
            Serial.print("\t");
        #endif
   }
}
