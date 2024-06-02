//Libraries 
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <PubSubClient.h>

//Initializing Particle Sensor
MAX30105 particleSensor;

//Define Constants
#define MAX_BRIGHTNESS 255
#define WIFISSID "Bahria Faculty" // Put your WifiSSID here
#define PASSWORD "bulc@fac-01" // Put your wifi password here
#define TOKEN "BBUS-llkCCBCGG4YJGJjsl12mywraAcfQkV" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "ApneaSense" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
                                           //it should be a random and unique ascii string and different from all other devices
#define DEVICE_LABEL "apneasense" // Assig the device label
#define VARIABLE_LABEL1 "spo2" // Assing the variable label
const int PUBLISH_FREQUENCY = 500; // Update rate in milliseconds



//Define Variables
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
unsigned long timer; //time variable

byte pulseLED = 16; //Must be on PWM pin
byte readLED = 17; //Blinks with each data read

char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[200];
char topic[150];


//Client Credentials
WiFiClient ubidots;
PubSubClient client(ubidots);

/****************************************
 *           Auxiliar Functions         *
 ****************************************/

//Function for Finger Detection
bool isFingerDetected() {
  long irValue = particleSensor.getIR();
  return irValue >= 50000; // Adjust this threshold as needed
}

//Call Back Function
void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  Serial.write(payload, length);
  Serial.println(topic);
}
 
 //Reconnect Function to MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

//setup
void setup()
{
 Serial.begin(115200); // initialize serial communication at 115200 bits per second:
 
 //Pin setup ESP32 for SpO2
 pinMode(pulseLED, OUTPUT);
 pinMode(readLED, OUTPUT);

 // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  // Waiting for user to Start coversation 
  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  //Setting Up parameters for SpO2 calculation
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  //Setting up particle sensor
   particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //Timer for Publish Frequency
   timer = millis();

  //Initializing Wifi & Server Setup 
  WiFi.begin(WIFISSID, PASSWORD);
  Serial.println();
  Serial.print("Waiting for WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);  
}
void loop(){
  //Condition to check whether Finger is placed on Sensor
  if(isFingerDetected()){

    bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

    //read the first 100 samples, and determine the signal range
    for (byte i = 0; i < bufferLength; i++) {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

     //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1) {

      if (!isFingerDetected()) {
        break; // exit the loop if the finger is not detected
      }

      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }

      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++) {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data

        digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample

        //send samples and calculation result to terminal program through UART
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);
      }

      //After gathering 25 new samples recalculate HR and SP02
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      // Publish data to Ubidots Logic
      if(validSPO2){

        if(!client.connected()){
          reconnect();
        }

        if(labs(millis() - timer) > PUBLISH_FREQUENCY)
        {
          sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
          sprintf(payload, "%s", ""); // Cleans the payload
          sprintf(payload, "{\"%s\": %d}", VARIABLE_LABEL1, spo2);

          Serial.println("Publishing data to Ubidots Cloud.");
          client.publish(topic, payload);
          timer = millis();
        }
      }
      client.loop();
    }  
  }
  else{
    Serial.println("Finger not detected !!");
    delay(1000); // Adjust delay as needed to avoid printing too frequently
  }
   client.loop();
}
