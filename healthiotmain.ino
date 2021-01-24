//Adding the heder to check PR.
// Connections : SCL PIN - D1 , SDA PIN - D2 , INT PIN - D0 , 
//PIR(OUT) - (ESP32 - GPIO 13, Voltage = 5V).
//LED - GPIO27
//Temp GPIO33, 3V

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include "FirebaseESP32.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>


#include <OneWire.h>
#include <DallasTemperature.h>

#define SDA 21
#define SCL 22

#define REPORTING_PERIOD_MS 1000
#define ECG_MONITORING_TIME 300000
#define REGISTER_BPM 1
#define REGISTER_OXYGEN 2
#define REGISTER_BODY_TEMP 3
#define REGISTER_DIST_VIOLATION_COUNT 4

#define FB_DB_BPM_SPO2_READ_STATUS "/S_BPM_SPO2_READ_STATUS"
#define FB_DB_BODY_TEMP_READ_STATUS "/S_BODY_TEMP_READ_STATUS"
#define FB_DB_DIST_VIOLATION__READ_STATUS "/S_DIST_VIOLATION__READ_STATUS"
#define FB_DB_ECG_READ_STATUS "/S_ECG_READ_STATUS"
#define FB_DB_USER_NAME "/UserName"

/*#define FB_DB_BMP_PATH "/lexio/SensorData/OxymeterReadings/PulseReading/Pulse"
#define FB_DB_OXYGEN_PATH "/lexio/SensorData/OxymeterReadings/OxygenReading/Oxygen:"
#define FB_DB_DIST_VIOLATION_PATH "/lexio/SensorData/SocialDistanceViolation/Record:"
#define FB_DB_BODY_FH_TEMP_PATH "/lexio/SensorData/Temperature/Record:"*/

#define FB_DB_BMP_PATH "/SensorData/OxymeterReadings/PulseReading/Pulse"
#define FB_DB_OXYGEN_PATH "/SensorData/OxymeterReadings/OxygenReading/Oxygen:"
#define FB_DB_DIST_VIOLATION_PATH "/SensorData/SocialDistanceViolation/Record:"
#define FB_DB_BODY_FH_TEMP_PATH "/SensorData/Temperature/Record:"

#define FIREBASE_HOST "lexhealthio.firebaseio.com"
#define FIREBASE_AUTH "C1EeEbvsh2Ogo9jrHpS9oFCdObv109jozjvmAQwR"
#define WIFI_SSID "MYSSID"
#define WIFI_PASSWORD "MYPASSWORD"

// Connections : SCL PIN - D1 , SDA PIN - D2 , INT PIN - D0
PulseOximeter pox;
 
float BPM, SpO2;

float averageBPM = 0;
float averageOxygen = 0;

float totalBPM = 0;
float totalOxygen = 0;

int counterBPM = 0;
int counterOxygen = 0;
int cycleCompleteCount = 0;
int old_cyclecount = 0;

String sDoReadBMP_SPO2 = "";
String sDoReadBMP_body_temp = "";
String sDoReadBMP_dist_viloation = "";
String sDoRead_ECG = "";
String sUserName = "";

uint32_t tsLastMilliReport = 0;
uint32_t tsSetMilliCounter = 0;

uint32_t initial_time = 0;
uint32_t current_time = 0;
uint32_t timegap = 0;


uint32_t global_initial_time = 0;
uint32_t global_current_time = 0;
uint32_t global_timegap = 0;

String sysTime;


int beatAvg;  
int checkForBeat = 0;
int chkstatus = 0;
int beat = 1;


int SENSOR_BPM_SP02_READY_FOR_AVERAGE = 0;
int SENSOR_BODY_TEMP_READY_FOR_AVERAGE = 0;
int SENSOR_DIST_VIOLATION_READY_FOR_AVERAGE = 0;

int realtime_looping_counter = 0;
int global_cyclecount = 0;
int ESP_Reset_Eligibility = 0;


float totalBODY_TEMP_FH = 0;
float BODY_TEMP_FH = 0;
int counterBODY_TEMP_FH = 0;
float averageBODY_TEMP_FH = 0;


bool BMP_SPO2_Connection = false;
bool do_Read_FB_DB_Checking_Read_State = false;

//Define FirebaseESP8266 data object.
FirebaseData firebaseData;


//*************************************For the Temperature Sensor*************************************
// GPIO where the DS18B20 is connected to.
// Connecting to GPIO33.
const int tempSensor_DS18B20_PIN = 33;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(tempSensor_DS18B20_PIN);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);


//**************************************** ECG Sensor ****************************************
//#define TOKEN "BBFF-ooWWXI72zrmAZ7zwol3kzaunzKj7VG" //Lex

#define TOKEN "BBFF-Z5oEYZZ8HKEkOXLT3sBbRImZIzdkgs" //AB
#define MQTT_CLIENT_NAME "lexhealthiot"                                            

#define VARIABLE_LABEL "sensor" // Assing the variable label
#define DEVICE_LABEL "esp32" // Assig the device label
 
#define ECG_SENSOR_PIN_CONNECTION A0 // Set the A0 as SENSOR
 
char mqttBroker[]  = "industrial.api.ubidots.com";
char payload[100];
char topic[150];
// Space to store values to send
char str_sensor[10];
 
WiFiClient ubidots;
PubSubClient client(ubidots);


void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  Serial.write(payload, length);
  Serial.println(topic);
}
 
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


//*************************************For the PIR Sensor*************************************
// GPIO where the HCSR501 is connected to.
// Connecting to D5.
const int pirSensor_HCSR501_PIN = 13;    
int PIR_CURRENT_VALUE = 0;
int totalDIST_VIOLATION_COUNT = 0;

int CONNECTED_LED_PIN = 27;
int PIR_SENSOR_STATE = LOW;




//************************************* BPM Beat detection callback *************************************
void onBeatDetected()
{
    Serial.println("\nPulse Detected!");
    beat = 1;
    checkForBeat = 1;
}
//************************************* BPM Beat detection callback *************************************




//*********************************************** setup() ***********************************************
void setup()
{
    Serial.println("\nConfiguraation Starting.....Please wait.");
    
    Serial.begin(115200);
    Wire.begin(SDA, SCL); 
    
    pinMode(pirSensor_HCSR501_PIN, INPUT);
    pinMode(CONNECTED_LED_PIN, OUTPUT);

    //Setting the Wi-Fi Connection.
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print(".");
      delay(300);
    }
    
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    //Setting up the Firebase DB.
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    //Firebase.reconnectWiFi(true);
    do_Read_FB_DB_Checking_Read_State = true;
    
 
    Serial.print("Now Initializing Pulse Oximeter..");
 
    if (!pox.begin())
    {
         Serial.println("FAILED");
         BMP_SPO2_Connection = false;
         Serial.println("Aveek Nike");
         Serial.println(BMP_SPO2_Connection);
    }
    else
    {
         Serial.println("SUCCESS");   
         BMP_SPO2_Connection = true;  
    }
     
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);


   //For ECG Sensor _ AD8232
   // Assign the pin as INPUT 
   pinMode(ECG_SENSOR_PIN_CONNECTION, INPUT); 
   client.setServer(mqttBroker, 1883);
   client.setCallback(callback); 
     
    
    delay(15000); 
    Serial.println("\nConfiguraation Completed.....");
 
}

//*********************************************** setup() ***********************************************


//************************************************ loop() ***********************************************

void loop() 
{
    if(do_Read_FB_DB_Checking_Read_State == true)
    {
        Serial.println("\nReading DB Status");
        pox.shutdown();
        doCheckReadStatusFromFB();
        pox.resume();
    
        do_Read_FB_DB_Checking_Read_State = false;
  
        //Start_Test
  
        Serial.println("\nAveek  sDoReadBMP_body_temp = ");
        Serial.println(sDoReadBMP_body_temp);
        
        Serial.println("\nAveek  sDoReadBMP_dist_viloation = ");
        Serial.println(sDoReadBMP_dist_viloation);
        
        Serial.println("\nAveek  sDoReadBMP_SPO2 = ");
        Serial.println(sDoReadBMP_SPO2);
  
        Serial.println("\nAveek  sDoRead_ECG = ");
        Serial.println(sDoRead_ECG);
  
        Serial.println("\nAveek  Username = ");
        Serial.println(sUserName);
  
        //End_Test
      
        if((sDoReadBMP_SPO2 == "ON") && (BMP_SPO2_Connection == true))
        {
          //If asked to read the Oxymeter status, will not be reading any other value to avoid synchronization issues.
          sDoReadBMP_body_temp = "OFF"; 
          sDoReadBMP_dist_viloation = "OFF";
        }
  
        if(sDoRead_ECG == "ON")
        {
          sDoReadBMP_SPO2 = "OFF";
          sDoReadBMP_body_temp = "OFF"; 
          sDoReadBMP_dist_viloation = "OFF";
        }      
      
    }
    
    if((sDoReadBMP_SPO2 == "ON") && (BMP_SPO2_Connection == true))
    {
      pox.update();
     // Serial.print("\nChecking for BMP_SPO2 data.");
      if (millis() - timegap > REPORTING_PERIOD_MS)
        {          
            check_BPM_SPO2_Status();
            SENSOR_BPM_SP02_READY_FOR_AVERAGE = 1;  
  
            realtime_looping_counter++;
            if(realtime_looping_counter > 25)
            {
                global_cyclecount++;
              
                //Average out the BPM_SP02 count.
                if(SENSOR_BPM_SP02_READY_FOR_AVERAGE == 1)
                {
                    checkAverage_BPM_SPO2_Status();                            
                    ESP.restart();
                    
                    //We are making the ESP ready for reset if global cycle count is > 5 and Average calculation was done.  
                    ESP_Reset_Eligibility = true;                   
                }
  
                //Resetting all counters to its initial value. Even do_Read_FB_DB_Checking_Read_State is set to true.
                resetAllCounters();             
            }
    
            timegap = millis();
        }
       else
        {
          //Serial.print("\n 10 ms time gap");
        }
    }
    else if((sDoReadBMP_body_temp == "ON") || (sDoReadBMP_dist_viloation == "ON"))
    {
      //Serial.print("\nChecking for Temperature or Distance Violation");
      
      if (millis() - timegap > REPORTING_PERIOD_MS)
        {        
            //Check if you need to read the body temperature.
            if(sDoReadBMP_body_temp == "ON")
            {
                check_Body_Temp_Status();
                SENSOR_BODY_TEMP_READY_FOR_AVERAGE = 1;     
            }
  
  
            //Check if you need to read the distance violation status.
            if(sDoReadBMP_dist_viloation == "ON")
            {
                check_Dist_Violation_Status();
                SENSOR_DIST_VIOLATION_READY_FOR_AVERAGE = 1;     
            }
  
            
            realtime_looping_counter++;
            if(realtime_looping_counter > 25)
            {
                global_cyclecount++;
  
                 //Average out the Body Temperature count.
                if(SENSOR_BODY_TEMP_READY_FOR_AVERAGE == 1)
                {
                  checkAverage_Body_Temp_Status();  
      
                  //We are making the ESP ready for reset if global cycle count is > 5 and Average calculation was done.  
                  ESP_Reset_Eligibility = true;                   
                }
  
  
                //Average out the Social Distance count.
                if(SENSOR_DIST_VIOLATION_READY_FOR_AVERAGE == 1)
                {
                  checkAverage_Distance_Violation_Status();  
      
                  //We are making the ESP ready for reset if global cycle count is > 5 and Average calculation was done.  
                  ESP_Reset_Eligibility = true;                   
                }
  
                
                resetAllCounters();                      
              }
  
            timegap = millis();
          }
        else
        {
          //Serial.print("\n 10 ms time gap");
        }        
      
    } //End of Body Temperature checking and distance violation.
    else if(sDoRead_ECG == "ON")
    {
      check_ECG_Status();
      Serial.println("ECG reading Completed");  

      global_cyclecount++;
      ESP_Reset_Eligibility = true;  
      resetAllCounters();
    }
    else
    {
      Serial.print("\nNo Reading");
    }


    //Just to display on change of a cycle.
    if(old_cyclecount != global_cyclecount)
    {
        Serial.print("\n***********************************************Completed cycle count number: ");
        Serial.print(old_cyclecount); 

        if(global_cyclecount == 2 && ESP_Reset_Eligibility)
        {
          //Restarting ESP after two cycles.
          Serial.print("\nReseting the ESP to clear out things");
          //ESP.restart();      
        }

        old_cyclecount = global_cyclecount;
    }  
}// End of loop()

//************************************************ loop() ***********************************************


//********************************************** ECG Status**********************************************
// Check the current ECG status.
void check_ECG_Status()
{
   uint32_t current_time = 0;
   uint32_t start_time = 0;


   current_time = millis();
   start_time = millis();
    
  while(current_time < start_time + ECG_MONITORING_TIME)
  {
    if (!client.connected()) 
    {
      reconnect();
    }
   
    sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
    sprintf(payload, "%s", ""); // Cleans the payload
    sprintf(payload, "{\"%s\":", VARIABLE_LABEL); // Adds the variable label
    
    float sensor = analogRead(ECG_SENSOR_PIN_CONNECTION); 
    
    /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
    dtostrf(sensor, 4, 2, str_sensor);
    
    sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor); // Adds the value
    Serial.println("Publishing data to Ubidots Cloud");
    client.publish(topic, payload);
    client.loop();
    delay(500);


    current_time = millis();
  }

  
  for(int i = 0; i<20; i++)
  {
     digitalWrite(CONNECTED_LED_PIN, HIGH);
     delay(100);
     digitalWrite(CONNECTED_LED_PIN, LOW);   
  }
      
  Serial.println("Completing the ECG reading");          
}

//********************************************** ECG Status**********************************************




//**************************************** BMP & SPO2 Measurement ***************************************

// Check the current BPM_SPO2 status.
void check_BPM_SPO2_Status()
{
    
    BPM = pox.getHeartRate();
    SpO2 = pox.getSpO2();

    Serial.print("\nCurrentBPM: ");
    Serial.print(BPM);

    Serial.print("\nCurrentOxygen: ");
    Serial.print(SpO2);
    
    if(BPM > 45)
    {
      totalBPM = totalBPM + BPM;
      counterBPM++;
     
      Serial.print("\n\n\n\n**********Start_Loop Values BPM**********\n");
      
      Serial.println("\nGlobal Cycle Count: ");
      Serial.println(global_cyclecount);
      
      Serial.println("\nLocal Cycle Count: ");
      Serial.println(realtime_looping_counter);
      
      Serial.print("\ntotalBPM: ");
      Serial.print(totalBPM);

      Serial.print("\ncounterBPM: ");
      Serial.print(counterBPM);
      
      Serial.print("\n***********End_Loop Values BPM***********\n");
    }
    else
    {
      Serial.print("\nBPM Recording not yet started.Please wait.\n");
    }


    if(SpO2 > 80)
    {
      totalOxygen = totalOxygen + SpO2;
      counterOxygen++;
     
      Serial.print("\n\n\n\n**********Start_Loop Values Oxygen**********\n");
      
      Serial.println("\nGlobal Cycle Count: ");
      Serial.println(global_cyclecount);
      
      Serial.println("\nLocal Cycle Count: ");
      Serial.println(realtime_looping_counter);

      Serial.print("\nOxygen Value: ");
      Serial.print(totalOxygen);

      Serial.print("\ncounterOxygen: ");
      Serial.print(counterOxygen);
      
      Serial.print("\n***********End_Loop Values Oxygen***********\n");

    }    
    else
    {
      Serial.print("\nOxygen Recording not yet started.Please wait.\n");
    }                         
}

//Check and register the average BPM_SPO2 status.
void checkAverage_BPM_SPO2_Status()
{
    Serial.print("\n -----------------------------Checking Average for Cycle Count: ");
    Serial.print(global_cyclecount); 
       
    averageBPM = totalBPM/counterBPM;
    averageOxygen = totalOxygen/counterOxygen;
    
    Serial.print("\n\n\n\n**********Start_Average Values**********\n");     
    
    Serial.print("\n averageBPM: ");
    Serial.print(averageBPM);
      
    Serial.print("\n averageOxygen: ");
    Serial.print(averageOxygen);
      
    Serial.print("\n***********End_Average Values***********\n");
    
    //Time to register the BPM Data.
    if(!averageBPM)
    {
      Serial.print("\n Could not register Average BPM Data in DB....too low !! Please try again");
      averageBPM = 0;
    }
    
    Serial.print("\n********************************\n");
    Serial.print("\nAdding Average BPM Data in DB with value: ");
    Serial.print(averageBPM);
    logDataInFireBase(REGISTER_BPM); 
    Serial.print("\n********************************\n");
    
    
    //Time to register the Oxygen Data.
    if(!averageOxygen)
    {
      Serial.print("\nCould not register Average Oxygen Data in DB....too low !! Please try again");
      averageOxygen = 0;
    }
    
    
    Serial.print("\n********************************\n");
    Serial.print("\nAdding Average Oxygen Data in DB with value: ");
    Serial.print(averageOxygen);
    logDataInFireBase(REGISTER_OXYGEN); 
    Serial.print("\n********************************\n");
}

//**************************************** BMP & SPO2 Measurement ***************************************



//###################################################### Body Temperature Measurement ######################################################
//###################################################### Body Temperature Measurement ######################################################


// Check the current body temperature status.
void check_Body_Temp_Status()
{
    sensors.requestTemperatures(); 
    BODY_TEMP_FH = sensors.getTempFByIndex(0);

    Serial.println("\nRecorded Body Temperature: ");
    Serial.print(BODY_TEMP_FH);
    
    Serial.println("\nGlobal Cycle Count: ");
    Serial.println(global_cyclecount);
    
    Serial.println("\nLocal Cycle Count: ");
    Serial.println(realtime_looping_counter);
        
    if( (BODY_TEMP_FH > 70) && (BODY_TEMP_FH < 110) )
    {    
        totalBODY_TEMP_FH = totalBODY_TEMP_FH + BODY_TEMP_FH;
        counterBODY_TEMP_FH++;
      
        Serial.print("\n\n\n\n**********Start_Loop Values BODY_TEMP**********\n");
           
        Serial.println("\nTemperature Value: ");
        Serial.print(BODY_TEMP_FH);
        Serial.println("ºF");
      
        Serial.print("\n\n\n\n**********End_Loop Values BODY_TEMP**********\n");   
    }  
    else
    {
      Serial.print("\nBody Temperature recording not yet started.Please wait.\n");
    }
}


//Check and register the average body temperature status.
void checkAverage_Body_Temp_Status()
{
    Serial.print("\n *********************Checking Average Temperature for Cycle Count: ");
    Serial.print(global_cyclecount); 
    
    
    averageBODY_TEMP_FH = totalBODY_TEMP_FH / counterBODY_TEMP_FH;
    
    
    Serial.print("\n**********Start_Temperature_Average Values**********\n");     
    
    Serial.print("\n averageBODY_TEMP_FH: ");
    Serial.print(averageBODY_TEMP_FH);
      
    Serial.print("\n***********End_Temperature_Average Values***********\n");
    
    //Time to register the body temperature Data.
    if(!averageBODY_TEMP_FH)
    {
        Serial.print("\n Could not register Average Body Temperature Data in DB....too low !! Please try again");
        averageBODY_TEMP_FH = 0;
    }
   else
   {
      Serial.print("\n********************************\n");
      Serial.print("\nAdded Average Body Temperature Data in DB with value: ");
      Serial.print(averageBODY_TEMP_FH);

      logDataInFireBase(REGISTER_BODY_TEMP); 
      Serial.print("\n********************************\n");
    }    
}

//****************************************** Body Temperature Measurement ******************************************




//***************************************** Distance Violation Measurement *****************************************

//Check distance violation status.
void check_Dist_Violation_Status()
{
    PIR_CURRENT_VALUE = digitalRead(pirSensor_HCSR501_PIN);

    if(PIR_CURRENT_VALUE == HIGH)
    {

      digitalWrite(CONNECTED_LED_PIN, HIGH);
      delay(100);

      if(PIR_SENSOR_STATE == LOW)
      {
        Serial.println("\n********************Motion Detected********************");
        
        totalDIST_VIOLATION_COUNT++;
        PIR_SENSOR_STATE = HIGH;
      }            
    }
    else 
    {
      digitalWrite(CONNECTED_LED_PIN, LOW); // Turning off the LED
      delay(200);             // 200 milliseconds delay 
      
      if (PIR_SENSOR_STATE == HIGH)
      {
        Serial.println("\n********************Motion Stopped********************");
        PIR_SENSOR_STATE = LOW;       // update the variable state into LOW
      }
    }    

    Serial.print("\n\n\n\n**********Start_Loop Distance Violation_Count**********\n");

    Serial.println("\nDistance Violation Current Status: ");
    Serial.print(PIR_CURRENT_VALUE);
    
    Serial.println("\nDistance Violation Count: ");
    Serial.print(totalDIST_VIOLATION_COUNT);
    
    
    Serial.print("\n\n\n\n**********End_Loop Distance Violation_Count**********\n");   
  
}


//Check and register average distance violation status.
void checkAverage_Distance_Violation_Status()
{
   //Time to register the Social Distance Violation status.
    if(!totalDIST_VIOLATION_COUNT)
    {
        Serial.print("\nNo distance violation record.");
        totalDIST_VIOLATION_COUNT = 0;
    }
   else
   {
      Serial.print("\n********************************\n");
      Serial.print("\nAdding Social Distance violation Data in DB with value: ");
      Serial.print(totalDIST_VIOLATION_COUNT);

      logDataInFireBase(REGISTER_DIST_VIOLATION_COUNT); 
      Serial.print("\n********************************\n");
    }
  
}


//***************************************** Distance Violation Measurement *****************************************




//******************************************* Read the status from FireBase ****************************************

//This function will check for which sensor value to read.
void doCheckReadStatusFromFB()
{
    //Do read the status if Pulse & O2 monitoring is required.

    if(Firebase.getString(firebaseData, FB_DB_BPM_SPO2_READ_STATUS))
    {
      sDoReadBMP_SPO2 = firebaseData.stringData();
      Serial.println("\nBMP_O2 Reading status from Fire Base: ");
      Serial.println(sDoReadBMP_SPO2); 
    }

    //Do read the status if body temperature monitoring is required.
    if(Firebase.getString(firebaseData, FB_DB_BODY_TEMP_READ_STATUS))
    {
      sDoReadBMP_body_temp = firebaseData.stringData();
      Serial.println("\nBody Temperature Reading status from Fire Base: ");
      Serial.println(sDoReadBMP_body_temp); 
    }
   
    //Do read the status if social distance violation monitoring is required.
    if(Firebase.getString(firebaseData, FB_DB_DIST_VIOLATION__READ_STATUS))
    {
      sDoReadBMP_dist_viloation = firebaseData.stringData();
      Serial.println("\nSocial Distance violation Reading status from Fire Base: ");
      Serial.println(sDoReadBMP_dist_viloation); 
    }

    //Do read the status if ECG monitoring is required.
    if(Firebase.getString(firebaseData, FB_DB_ECG_READ_STATUS))
    {
      sDoRead_ECG = firebaseData.stringData();
      Serial.println("\nECG Reading status from Fire Base: ");
      Serial.println(sDoRead_ECG); 
    }
    

    //Read the Username.
    if(Firebase.getString(firebaseData, FB_DB_USER_NAME))
    {
      sUserName = firebaseData.stringData();
      Serial.println("\nRegistered User Name: ");
      Serial.println(sUserName); 
    }

    if (sUserName == "")
    {
      sUserName = "Unknown User";     
    }    

}

//******************************************* Read the status from FireBase ****************************************


//***************************************** Reset all the counter values *******************************************

//Reset all counter values to what it was during the initial time.
void resetAllCounters()
{
    //Now is the time to reset all data for the fresh cycle.      
    totalBPM = 0;
    averageBPM = 0;
    counterBPM = 0;
     
    totalOxygen = 0;
    averageOxygen = 0;
    counterOxygen = 0;  

    chkstatus = 0; 
    realtime_looping_counter = 0;

    //ESP_Reset_Eligibility = 0;
    SENSOR_BPM_SP02_READY_FOR_AVERAGE = 0;
    SENSOR_BODY_TEMP_READY_FOR_AVERAGE = 0;
    SENSOR_DIST_VIOLATION_READY_FOR_AVERAGE = 0;

    sDoReadBMP_SPO2 = "";
    sDoReadBMP_body_temp = "";
    sDoReadBMP_dist_viloation = "";
    sUserName = "";

    totalBODY_TEMP_FH = 0;
    BODY_TEMP_FH = 0;
    counterBODY_TEMP_FH = 0;
    averageBODY_TEMP_FH = 0;

    PIR_CURRENT_VALUE = 0;
    totalDIST_VIOLATION_COUNT = 0;

    do_Read_FB_DB_Checking_Read_State = true;

}

//***************************************** Reset all the counter values *******************************************



//**************************************** Update the Firebase DB with data ****************************************

void logDataInFireBase(int regIndex)
{
    String fb_DB_Path;
    float fb_Value = 0;
    sysTime = getTime();

    if(regIndex == REGISTER_BPM)
    {
        fb_DB_Path = FB_DB_BMP_PATH;
        fb_Value = averageBPM;
    }
    
    if(regIndex == REGISTER_OXYGEN)
    {
        fb_DB_Path = FB_DB_OXYGEN_PATH;
        fb_Value = averageOxygen;
    }

    if(regIndex == REGISTER_BODY_TEMP)
    {
        fb_DB_Path = FB_DB_BODY_FH_TEMP_PATH;
        fb_Value = averageBODY_TEMP_FH;
    }

    if(regIndex == REGISTER_DIST_VIOLATION_COUNT)
    {
        fb_DB_Path = FB_DB_DIST_VIOLATION_PATH;
        fb_Value = totalDIST_VIOLATION_COUNT;
    }

    //Ad the user name and the timing.
    //fb_DB_Path = sUserName + fb_DB_Path + sysTime;

    String path = "/lexio/" + sUserName + fb_DB_Path + sysTime;
    Serial.print("\nAdding the data in Firebase Path\n");
    Serial.print(path);
    Serial.print("\n");
    Serial.print(fb_DB_Path);
    
    Serial.print("\nValue Added: \n");
    Serial.print(fb_Value);

       
    Firebase.setFloat(firebaseData,"/lexio/" + sUserName + fb_DB_Path + sysTime, fb_Value);  
    
    //Firebase.setFloat(firebaseData,"/lexio/" + fb_DB_Path, fb_Value);  
}

//**************************************** Update the Firebase DB with data ****************************************


//********************************************* Calculate current time *********************************************

//Calculate the current time.
String getTime() {
  WiFiClient client;
  while (!!!client.connect("google.com", 80)) {
    Serial.println("connection failed, retrying...");
  }

  client.print("HEAD / HTTP/1.1\r\n\r\n");

  while (!!!client.available()) {
    yield();
  }

  while (client.available()) {
    if (client.read() == '\n') {
      if (client.read() == 'D') {
        if (client.read() == 'a') {
          if (client.read() == 't') {
            if (client.read() == 'e') {
              if (client.read() == ':') {
                client.read();
                String theDate = client.readStringUntil('\r');
                client.stop();
                return theDate;
              }
            }
          }
        }
      }
    }
  }
}

//********************************************* Calculate current time *********************************************
