#include <Adafruit_PWMServoDriver.h>
// #include <BluetoothSerial.h>
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#define SMOOTHING_FACTOR 0.3 // Adjust between 0 (no smoothing) and 1 (no filtering)

#define VRX_PIN 36 // X-axis analog pin
#define VRY_PIN 39 // Y-axis analog pin
#define VRZ_PIN 34 // Z-axis analog pin

#define SERVOMIN1 150
#define SERVOMAX1 600
#define SERVOMIN2 150
#define SERVOMAX2 600 

#define SERVOMIND 150.0
#define SERVOMAXD 600.0

#define shoulderLink 12.5
#define elbowLink    12.5

// Wi-Fi credentials
const char* ssid = "ESP32_AP1";
const char* password = "12345678";
const int channel = 1;
WiFiServer server(12345);  // Create a server on port 12345
WiFiClient client;  // Declare the client

// Servo channel definitions
#define SER0 0
#define SER1 1
#define SER2 2
#define SER3 4
#define SER4 3
#define SER5 5
#define SER6 6

bool playRecording  = false;
bool startRecording = false;
bool saveRecording  = false;

//variables to store currentServoAngles
int currentShoulderPwm = 0;
int currentElbowPwm    = 0;
int currentBasePwm     = 0;
int currentRollPwm     = 0;
int currentPitchPwm    = 0;
int currentClawPwm     = 0;

int xFiltered = 0;
int yFiltered = 0;
int zFiltered = 0;

//BluetoothSerial btObject;
Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

float xValue = 0;
float yValue = 0;
float zValue = 0.0; 

int neutralX=0;
int neutralY=0;
int neutralZ=0;

int xRaw ;
int yRaw;
int zRaw;
// Dead zone threshold to ignore joystick jitter
const int DEAD_ZONE = 100;

TaskHandle_t taskHandleWifi     ;
TaskHandle_t taskRecieveDataBL  ;
TaskHandle_t taskRecieveDataJst ;
TaskHandle_t taskMoveAssembly   ;
TaskHandle_t taskCalculateAngles;
TaskHandle_t taskMoveAssemblyJst;
TaskHandle_t taskTakeSerialInput;   
TaskHandle_t taskPlayRecording  ;
TaskHandle_t taskRecordData     ;

QueueHandle_t moveAssemblyQueue       ;
QueueHandle_t joystickCoordinatesQueue;
QueueHandle_t moveAssemblyJstQueue    ;
QueueHandle_t moveAssemblyWCQueue     ;
QueueHandle_t servoMoveQueue          ;
QueueHandle_t recordDataQueue         ;

TimerHandle_t servoUpdateTimer;

SemaphoreHandle_t calculateAnglesMutex    ;
SemaphoreHandle_t moveBodyArmAssemblyMutex;

struct angles
{
float baseTheta    ;
float shoulderTheta;
float elbowTheta   ;
float pitchTheta   ;
float rollTheta    ;
float clawTheta    ;
};

//strcuture to form DoublylinkedList for recording!
struct node
{
  struct node   *nextNode      ;
  struct angles *anglesRecorded;
  struct node   *prevNode      ;
};
//pointers for struct node
struct node* rootNode = NULL;
struct node* tempNode = NULL;

enum quadrant{
  firstQuadrant  = 1,
  secondQuadrant = 2,
  thirdQuadrant  = 3,
  fourthQuadrant = 4
};


struct dataPacket{
  float xValue ;
  float yValue ;
  float zValue ;
  float pitch  ;
  float roll   ;
  float clawVal;
};
struct dataPacket dataPacketRecieved;

//int i =0;

struct servoData
{
  int servoNumber;
  int currentPwm ;
  int targetPwm  ;
};

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//callback ISR function exectutes during communication(reception) over ESP NOW
void onDataRecv(const uint8_t *mac, const uint8_t *dataPacketIncoming, int len)
{
    // Ensure the received data length matches the expected structure size
    if (len == sizeof(dataPacketRecieved))
    {  // Copy the received data into the dataPacketRecieved structure
        memcpy(&dataPacketRecieved, dataPacketIncoming, sizeof(dataPacketRecieved));

        // Variable to track if a higher-priority task needs to be woken
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        // startRecording = dataPacketRecieved.startRecording;
        // saveRecording  = dataPacketRecieved.saveRecording ;
        // playRecording  = dataPacketRecieved.playRecording ;  
            
        // Try to enqueue the data onto the queue
        if (xQueueSendFromISR(
         moveAssemblyWCQueue,
         &dataPacketRecieved, 
         &xHigherPriorityTaskWoken
         ) != pdTRUE)
        {// Log an error if the queue is full
           Serial.println("Failed to enqueue onto moveAssemblyWC Queue");
        }

        
     // Perform a context switch if a higher-priority task was woken
       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {// Log an error if the received data length is invalid
      Serial.println("Received data length does not match expected size");
    }
}


// void parsing(String dataa) {
//   dataPacket dataPacketCV;

//   String subString1 = "", subStringM = "", subString3 = "";

//   // Parse based on the expected format
//   if (dataa.length() >= 5) { // Ensure minimum length for parsing
//     if (dataa.indexOf('-') == 0) {
//       subString1 = dataa.substring(0, 3);
//       subStringM = dataa.substring(3, 6);
//       subString3 = dataa.substring(6);
//     } else {
//       subString1 = dataa.substring(0, 2);
//       subStringM = dataa.substring(2, 5);
//       subString3 = dataa.substring(5);
//     }
//   } else {
//     Serial.println("Error: Insufficient data length");
//     return; // Exit parsing if data is invalid
//   }

//   // Validate and convert substrings
//   static float xSum = 0, ySum = 0, zSum = 0;
//   static int i = 0;

//   if (isDigit(subString1.charAt(0)) && isDigit(subStringM.charAt(0)) && isDigit(subString3.charAt(0))) {
//     xSum += float(subString1.toInt());
//     ySum += float(subStringM.toInt());
//     zSum += float(subString3.toInt());
//     i++;
//   } else {
//     Serial.println("Error: Invalid numeric data");
//     return;
//   }

//   // Calculate and send averages after 10 readings
//   if (i >= 10) {
//     dataPacketCV.xValue = xSum / 10.0;
//     dataPacketCV.yValue = ySum / 10.0;
//     dataPacketCV.zValue = zSum / 10.0;

//     Serial.print(" X val : ");
//     Serial.println(dataPacketCV.xValue);
//     Serial.print(" Y val : ");
//     Serial.println(dataPacketCV.yValue);
//     Serial.print(" Z val : ");
//     Serial.println(dataPacketCV.zValue);

//     xSum = 0;
//     ySum = 0;
//     zSum = 0;
//     i = 0;

//     if (xQueueSend(moveAssemblyWCQueue, &dataPacketCV, portMAX_DELAY) != pdTRUE) {
//       Serial.println("Failed to enqueue onto moveAssemblyWCQueue");
//     }
//   }
// }


// void parsing(String data) {
//     //ParsedData parsed;
//     dataPacket dataPacketCV;
    
//     int firstSpace = data.indexOf(' ', 7);  // Find first space after "Data : "
//     int secondSpace = data.indexOf(' ', firstSpace + 1); // Find second space
//     // Extract substrings and convert to float
//     float yVal1 = data.substring(7, firstSpace).toFloat();
//     float zVal1 = data.substring(firstSpace + 1, secondSpace).toFloat();
//     float xVal1 = data.substring(secondSpace + 1).toFloat();
    
//       // Static variables to hold cumulative sums and counter
//   static float xSum = 0, ySum = 0, zSum = 0;
//   static  int i = 0;

//   // Accumulate parsed values
//   xSum += xVal1;
//   ySum += yVal1;
//   zSum += zVal1;
//   i++;

//    if (i >= 2) { // After collecting 2 readings
//      dataPacketCV.xValue = xSum / 2;
//      dataPacketCV.yValue = ySum / 2;
//      dataPacketCV.zValue = zSum / 2;

//     // Print averaged values
//     Serial.print(" X Sum : ");
//     Serial.print(dataPacketCV.xValue);
//     Serial.print(" Y Sum : ");
//     Serial.print(dataPacketCV.yValue);
//     Serial.print(" Z Sum : ");
//     Serial.println(dataPacketCV.zValue);

//     // Reset sums and counter
//     xSum = 0;
//     ySum = 0;
//     zSum = 0;
//     i = 0;

//     // Enqueue the averaged data
//     if (xQueueSend(moveAssemblyWCQueue,
//          &dataPacketCV,
//        portMAX_DELAY) != pdTRUE) {
//       Serial.println("Failed to enqueue onto moveAssemblyWCQueue");
//     }
//   }
// }

void parsing(String data) {
    dataPacket dataPacketCV;

    // Find positions of spaces
    int firstSpace = data.indexOf(' ');  // First space
    int secondSpace = data.indexOf(' ', firstSpace + 1); // Second space

    // Check if spaces exist to prevent errors
    if (firstSpace == -1 || secondSpace == -1) {
        Serial.println("Invalid data format!");
        return;
    }

    // Extract substrings and convert to float
    float xVal1 = data.substring(0, firstSpace).toFloat();
    float yVal1 = data.substring(firstSpace + 1, secondSpace).toFloat();
    float zVal1 = data.substring(secondSpace + 1).toFloat();

    // Static variables to hold cumulative sums and counter
    static float xSum = 0, ySum = 0, zSum = 0;
    static int i = 0;

    // Accumulate parsed values
    xSum += xVal1;
    ySum += yVal1;
    zSum += zVal1;
    i++;

    if (i >= 2) { // After collecting 2 readings
        dataPacketCV.xValue = xSum / 2;
        dataPacketCV.yValue = ySum / 2;
        dataPacketCV.zValue = zSum / 2;

        // Print averaged values
        Serial.print("X Avg: ");
        Serial.print(dataPacketCV.xValue);
        Serial.print(" Y Avg: ");
        Serial.print(dataPacketCV.yValue);
        Serial.print(" Z Avg: ");
        Serial.println(dataPacketCV.zValue);

        // Reset sums and counter
        xSum = 0;
        ySum = 0;
        zSum = 0;
        i = 0;

        // Enqueue the averaged data
        if (xQueueSend(moveAssemblyWCQueue, &dataPacketCV, portMAX_DELAY) != pdTRUE) {
            Serial.println("Failed to enqueue onto moveAssemblyWCQueue");
        }
    }
}


// void parsing(String dataa) {
//   // Parsing logic`
//   dataPacket dataPacketCV;

//   String subString1, subString2, subString3, subStringM;

//   if (dataa.indexOf('-') == 0) {
//     subString1 = dataa.substring(0, 3);
//     subString2 = dataa.substring(3, 5);
//     if (subString2.indexOf('-') == 0) {
//       subStringM = dataa.substring(3, 6);
//       subString3 = dataa.substring(5);
//     }
//   } else {
//     subString1 = dataa.substring(0, 2);
//     subString2 = dataa.substring(2, 4);
//     if (subString2.indexOf('-') == 0) {
//       subStringM = dataa.substring(2, 5);
//       subString3 = dataa.substring(5);
//     } else {
//       subStringM = dataa.substring(2, 4);
//       subString3 = dataa.substring(4);
//     }
//   }

//   // Static variables to hold cumulative sums and counter
//   static float xSum = 0, ySum = 0, zSum = 0;
//   static int i = 0;

//   // Accumulate parsed values
//   xSum += subString1.toInt();
//   ySum += subStringM.toInt();
//   zSum += subString3.toInt();
//   i++;

//    if (i >= 5) { // After collecting 10 readings
//      dataPacketCV.xValue = xSum / 5;
//      dataPacketCV.yValue = ySum / 5;
//      dataPacketCV.zValue = zSum / 5;

//     // Print averaged values
//     Serial.print(" X Sum : ");
//     Serial.println(dataPacketCV.xValue);
//     Serial.print(" Y Sum : ");
//     Serial.println(dataPacketCV.yValue);
//     Serial.print(" Z Sum : ");
//     Serial.println(dataPacketCV.zValue);

//     // Reset sums and counter
//     xSum = 0;
//     ySum = 0;
//     zSum = 0;
//     i = 0;

//     // Enqueue the averaged data
//     if (xQueueSend(moveAssemblyWCQueue, &dataPacketCV, portMAX_DELAY) != pdTRUE) {
//       Serial.println("Failed to enqueue onto moveAssemblyWCQueue");
//     }
//   }
// }


// Task to handle Wi-Fi communications
void handleWifiTask(void *pvParameters) {
    static WiFiClient client;  // Declare the client outside the loop to persist across iterations
    while (1) {
        // Check if a new client is available
        if (!client || !client.connected())
         {
            client = server.available();  // Accept a new connection if no active client
            Serial.println("Trying to reconnect client!");
         }
        if (client) {
            //Serial.println("Client connected!");           
            String incomingData = "";
            // Process the connected client
            while (client.connected()) {
                if (client.available()) {
                    char c = client.read();  // Read the incoming byte
                    // If it's a new line (indicating end of the command)
                    if (c == '\n') {
                      //  Serial.print("Received: ");
                        //Serial.println(incomingData);  // Print received data  
                        parsing(incomingData);
                        // Process the data or handle commands here
                        incomingData = "";  // Clear the buffer for next data
                    } else {
                        incomingData += c;  // Append the character to the string
                    }
                }
            }   
           // Serial.println("Client disconnected!");
            //client.stop();  // Close the client connection after data is processed
           //vTaskDelay(pdMS_TO_TICKS(1));
        } 
        else 
        {
          Serial.println("Not connected!");
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Avoid busy waiting
    }
}

void takeSerialInputTask(void *pvParameters) { 
    struct dataPacket serialPacket;
    while (1) {
        if (Serial.available() > 0) {
            // Read input from the Serial monitor
      
            String input = Serial.readStringUntil('\n'); // Read input from Serial Monitor
            Serial.print("Input is: ");
            Serial.println(input);
            String subString1, subString2, subString3,subStringM;
            int x, y, z;

            if(input.indexOf('-') == 0)
            { //reading the negative x value
              Serial.println("x is negative!!");
              subString1 = input.substring(0,3);
              subString2 = input.substring(3,5);

              if(subString2.indexOf('-') == 0)
              {//f,or negative y value
              Serial.println("Y is negative!");
              subStringM = input.substring(3,6);
              subString3 = input.substring(6)  ;
              }
              else
              {//for positive yValue
               Serial.println("Y is positive!");
               subStringM = input.substring(3,5);
               subString3 = input.substring(5);
              }
            }
            else{
              //reading positive X value
              Serial.println("X is positive!");
              subString1 = input.substring(0,2);
              subString2 = input.substring(2,4);

              if(subString2.indexOf('-') == 0)
              {//for negative y value
              Serial.println("Y is negative!");
              subStringM = input.substring(2,5);
              subString3 = input.substring(5)  ;
              }
              else
              {//for positive yValue
              Serial.println("y is positive!");
               subStringM = input.substring(2,4);
               subString3 = input.substring(4)  ;
              }
            }
            // Extract and convert substrings
            serialPacket.xValue = subString1.toInt(); // First number (x)
            serialPacket.yValue = subStringM.toInt(); // Second number (y)
            serialPacket.zValue = subString3.toInt(); // Third number (z)
                
                if(serialPacket.yValue == 0 && serialPacket.xValue == 0 && serialPacket.zValue < 10)
                {
                  serialPacket.zValue = 10;
                }
        // queue send             
        if(xQueueSend(
          moveAssemblyWCQueue,
          &serialPacket,
          portMAX_DELAY
          ) != pdTRUE)
          Serial.println("Failed to send onto queue!");

        } 
        // Check again after 10 milliseconds
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void calculateAnglesTask(void *pvParameters)
{
    float thetaBaseServo, thetaShoulderServo, thetaElbowServo, thetaPitchServo, thetaRollServo, thetaClawServo;
    struct dataPacket coordinatesReceived;

    while (1)
    {  
      if (xQueueReceive(
          moveAssemblyWCQueue,
          &coordinatesReceived,
          portMAX_DELAY) == pdTRUE)
        {
            // Extract coordinates
            float x = coordinatesReceived.xValue;
            float y = coordinatesReceived.yValue;
            float z = coordinatesReceived.zValue;
            float pitch = coordinatesReceived.roll;
            float roll = coordinatesReceived.pitch;
            float claw = coordinatesReceived.clawVal;

            // Ensure Z is at least 10 cm if conditions are met
            if (x <= 3.0 && y <= 0.0 && z < 10.0)
            {
                z = 10.0;
            }

            // Calculate base angle
            float thetaBaseRadians = atan2(y, x);
            thetaBaseServo = thetaBaseRadians * 180.0 / PI;

            // Displacement magnitude calculations
            float dispXYMgtd = sqrt(pow(x, 2) + pow(y, 2));
            float dispYZMgtd = sqrt(pow(dispXYMgtd, 2) + pow(z, 2));

            // Check if the target is reachable
            if (dispYZMgtd > (shoulderLink + elbowLink))
            {
                Serial.println("Target out of reach");
                continue;
            }

            // Direction angle in Y-Z plane
            float dispYZDirRadians = atan2(z, dispXYMgtd);
            float dispYZDir = dispYZDirRadians * 180.0 / PI;

            // Calculate shoulder and elbow angles using the law of cosines
            float cosShoulderAngle = (pow(shoulderLink, 2) + pow(dispYZMgtd, 2) - pow(elbowLink, 2)) / (2 * shoulderLink * dispYZMgtd);
            float thetaShoulderServoRadians = acos(cosShoulderAngle);
            thetaShoulderServo = (thetaShoulderServoRadians * 180.0 / PI) + dispYZDir;

            float cosElbowAngle = (pow(shoulderLink, 2) + pow(elbowLink, 2) - pow(dispYZMgtd, 2)) / (2 * shoulderLink * elbowLink);
            float thetaElbowServoRadians = acos(cosElbowAngle);
            thetaElbowServo = 180.0 - (thetaElbowServoRadians * 180.0 / PI);

            // Map claw angle
            thetaClawServo = mapFloat(claw, 20.0, 0.0, 85.0, 0.0);

            // Map pitch servo angle
            if (pitch >= 0.3 && pitch <= 90.0)
            {
                thetaPitchServo = 90.0 + pitch;
            }
            else if (pitch <= -0.3 && pitch >= -100.0)
            {
                thetaPitchServo = 90.0 + pitch;
            }

            // Map roll servo angle
            if (roll >= 0.0 && roll <= 90.0)
            {
                thetaRollServo = roll;
            }

            // Print results
            // Serial.print("Base servo angle: ");
            // Serial.println(thetaBaseServo);
            // Serial.print("Shoulder servo angle: ");
            // Serial.println(thetaShoulderServo);
            // Serial.print("Elbow servo angle: ");
            // Serial.println(thetaElbowServo);
            // Serial.print("Claw Servo Angle: ");
            // Serial.println(thetaClawServo);
            // Serial.print("Pitch Servo Angle: ");
            // Serial.println(thetaPitchServo);

            // Constrain servo angles
            float thetaConstShoulderServo = constrain(thetaShoulderServo, 0.0, 180.0);
            float thetaConstElbowServo = constrain(thetaElbowServo, 0.0, 126.0);
            float thetaConstClawServo = constrain(thetaClawServo, 0.0, 68.0);
            float thetaConstPitchServo = constrain(thetaPitchServo, 45.0, 180.0);
            thetaBaseServo = 180.0 -  thetaBaseServo;

            // Populate the angles structure
            struct angles anglesSent;
            anglesSent.baseTheta = thetaBaseServo;
            anglesSent.shoulderTheta = thetaConstShoulderServo;
            anglesSent.elbowTheta = thetaConstElbowServo;
            anglesSent.pitchTheta = thetaConstPitchServo;
            anglesSent.rollTheta = thetaRollServo;
            anglesSent.clawTheta = thetaConstClawServo;

           // Send angles to the move assembly queue
            if (anglesSent.baseTheta >= 0.0 && anglesSent.shoulderTheta >= 0.0)
            {
                if (xQueueSend(moveAssemblyJstQueue,
                               &anglesSent,
                               portMAX_DELAY) != pdTRUE)
                {
                    Serial.println("Failed to enqueue onto moveAssemblyQueue!");
                }
                // else
                // {
                //  // Serial.println("Successfully enqueued onto moveAssemblyQueue!");
                // }
            }
            else
            {
                Serial.println("Invalid angles calculated; skipping enqueue.");
            }
        }
    }
}

            // // Send angles to the move assembly queue
            // if (anglesSent.baseTheta >= 0.0 && anglesSent.shoulderTheta >= 0.0)
            // {
            //     if (xQueueSend(moveAssemblyQueue, &anglesSent, portMAX_DELAY) != pdTRUE)
            //     {
            //         Serial.println("Failed to enqueue onto moveAssemblyQueue!");
            //     }
            //     else
            //     {
            //         Serial.println("Successfully enqueued onto moveAssemblyQueue!");
            //     }

            //     // Record data if recording is active
            //     if (startRecording)
            //     {
            //         if (xQueueSend(recordDataQueue, &anglesSent, portMAX_DELAY) != pdTRUE)
            //         {
            //             Serial.println("Failed to enqueue onto recordDataQueue!");
            //         }
            //         else
            //         {
            //             Serial.println("Successfully enqueued onto recordDataQueue!");
            //         }
            //     }
            // }
            // else
            // {
            //     Serial.println("Invalid angles calculated; skipping enqueue.");
            // }
//         }
//     }
// }

// void smoothMove(int servo, int currentValue, int targetValue) {
//     int step = (currentValue < targetValue) ? 1 : -1;
//     for (int value = currentValue; value != targetValue; value += step) {
//         driver.setPWM(servo, 0, value);
//         if(servo == SER3 || servo == SER4 || servo == SER5){
//         //Serial.println("2 millisecond!!!!1");
//         vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for smooth movement
//         }
//         else if(servo == SER2)
//         { //for elbow servo
//           //Serial.println("Elbow @ 1 millis!");
//           vTaskDelay(pdMS_TO_TICKS(1));
//         }
//         else if(servo == SER0)
//         { //for base servo
//           //Serial.println("Base @ 1 millis!!!1");
//           vTaskDelay(pdMS_TO_TICKS(1));
//         }
//         else{
//         //For shoulder servo
//         // Serial.println(" Shoulder @ 1 millis!!!!!!!");
//          vTaskDelay(pdMS_TO_TICKS(1));
//         }
//     }
// }


// void smoothMove(int servo, int currentValue, int targetValue) {
//     const float decelerationFactor = 0.3; // Factor to blend the current and target values
//     int step = (currentValue < targetValue) ? 1 : -1;

//     while (currentValue != targetValue) {
//         // Calculate the next value, blending the current and target values
//         int nextValue = (int)(currentValue * decelerationFactor + targetValue * (1 - decelerationFactor));

//         // Ensure the next value moves in the correct direction
//         if ((step > 0 && nextValue > targetValue) || (step < 0 && nextValue < targetValue)) {
//             nextValue = targetValue;
//         }

//         // Set the servo PWM signal
//         driver.setPWM(servo, 0, nextValue);

//         // Update currentValue for the next iteration
//         currentValue = nextValue;

//         // Add a delay for smooth movement
//         if (servo == SER3 || servo == SER4 || servo == SER5) {
//             vTaskDelay(pdMS_TO_TICKS(1));
//         } else if (servo == SER2) { // For elbow servo
//             vTaskDelay(pdMS_TO_TICKS(1));
//         } else if (servo == SER0) { // For base servo
//             vTaskDelay(pdMS_TO_TICKS(1));
//         } else { // For shoulder servo
//             vTaskDelay(pdMS_TO_TICKS(1));
//         }
//     }
// }

void smoothMove(int servo, int currentValue, int targetValue) {
    while (currentValue != targetValue) {
        // Calculate the remaining distance between current position and target
        float distance = abs(targetValue - currentValue);
        // Dynamically adjust the deceleration factor based on distance (as distance decreases, the factor reduces)
        float decelerationFactor = 0.99 - 0.08 * (distance / 90.0); // Tweak this for smoother deceleration
        // Debug: Print the deceleration factor and distance for monitoring
       // Serial.print("Deceleration factor at distance ");
        //Serial.print(distance);
        //Serial.print(" is: ");
        //Serial.println(decelerationFactor);
        // Compute the next angle position using deceleration factor
        int nextValue = (int)(currentValue * decelerationFactor + targetValue * (1 - decelerationFactor));
        // Check if nextValue is too small, and adjust it to prevent tiny movement steps being ignored
        if (nextValue == currentValue) {
            nextValue = currentValue + (currentValue < targetValue ? 1 : -1);  // Step one unit if no change
        }
        // Apply the movement to the servo
        driver.setPWM(servo, 0, nextValue);
        // Update the current position for the next iteration
        currentValue = nextValue;
        
         if (servo == SER3 || servo == SER4 || servo == SER5) {
             vTaskDelay(pdMS_TO_TICKS(1));
         } else if (servo == SER2) { // For elbow servo
             vTaskDelay(pdMS_TO_TICKS(2));
         } else if (servo == SER0) { // For base servo
             vTaskDelay(pdMS_TO_TICKS(3));
         } else { // For shoulder servo
             vTaskDelay(pdMS_TO_TICKS(1));
         }
        
        // Delay for smooth movement (adjust the delay as necessary)
        //vTaskDelay(pdMS_TO_TICKS(1));
    }
}


// void smoothMove(int servo, int currentValue, int targetValue) {
//     const float decelerationFactor = 0.99; // Factor to blend the current and target values

//     while (currentValue != targetValue) {
//         // Calculate the next value, blending the current and target values
//         float nextValue = (float)(currentValue * decelerationFactor + targetValue * (1 - decelerationFactor));

//         // If the nextValue is the same as the currentValue, directly set the target to break out of the loop
//         if (nextValue == currentValue) {
//             nextValue = targetValue;
//         }
//         // Set the servo PWM signal
//         driver.setPWM(servo, 0, nextValue);
//         // Update currentValue for the next iteration
//         currentValue = nextValue;
//         // Add a delay for smooth movement
//         //vTaskDelay(pdMS_TO_TICKS(1));
//          // Add a delay for smooth movement
//          if (servo == SER3 || servo == SER4 || servo == SER5) {
//              vTaskDelay(pdMS_TO_TICKS(1));
//          } else if (servo == SER2) { // For elbow servo
//              vTaskDelay(pdMS_TO_TICKS(5));
//          } else if (servo == SER0) { // For base servo
//              vTaskDelay(pdMS_TO_TICKS(5));
//          } else { // For shoulder servo
//              vTaskDelay(pdMS_TO_TICKS(5));
//          }
//         //vTaskDelay(pdMS_TO_TICKS(5));
//     }
// }


// void recordDataTask(void *pvParameters)
// { struct angles anglesGot;
//   while(1)
//   {
//     if(xQueueReceive(
//      recordDataQueue, 
//      &anglesGot     ,
//      portMAX_DELAY
//     ) == pdPASS)
//     {//copy onto linked list
//           struct node* newNode=(struct node*)malloc(sizeof(struct node));
//            newNode -> anglesRecorded = &anglesGot;
//            newNode -> nextNode       = NULL       ;
//            newNode -> prevNode       = NULL       ;
           
//            Serial.println("****RECORDED*****");
//            Serial.print(" base Angle : ");
//            Serial.print(newNode->anglesRecorded->baseTheta);
//            Serial.print(" shoulder : ");
//            Serial.print(newNode->anglesRecorded->shoulderTheta);
//            Serial.print(" Elbow : ");
//            Serial.print(newNode->anglesRecorded->elbowTheta);
//            Serial.println("******************");
          
//            if(rootNode == NULL)
//             {
//               rootNode = tempNode = newNode;
//               newNode -> prevNode = NULL;
//               newNode -> nextNode = NULL; 
//             }
//            else 
//             {
//              tempNode -> nextNode = newNode;
//              newNode  -> prevNode = tempNode;
//              //update the tempNode
//               tempNode = newNode;
//             }
//             //wait 5 milliseconds to allow other tasks to run
//             vTaskDelay(5);
//     }
//   }
// }

void moveAssemblyJstTask(void *pvParameters)
{
    struct angles receivedAngles;
    while (1)
    {
        //Serial.println("MOVING BODYARM!!1");
         if (xQueueReceive(
                moveAssemblyJstQueue,
                &receivedAngles,
                portMAX_DELAY) == pdTRUE)
        {
    
            int pwmShoulderValue = mapFloat(receivedAngles.shoulderTheta, 0, 270, SERVOMIND, SERVOMAXD);
            int pwmElbowValue    = mapFloat(receivedAngles.elbowTheta   , 0, 180, SERVOMIND, SERVOMAXD);
            int pwmBaseValue     = mapFloat(receivedAngles.baseTheta    , 0, 180, SERVOMIN1, SERVOMAX1);
            int pwmRollValue     = mapFloat(receivedAngles.rollTheta    , 0, 180, SERVOMIN2, SERVOMAX2);             
            int pwmPitchValue    = mapFloat(receivedAngles.pitchTheta   , 0, 180, SERVOMIN2, SERVOMAX2);
            int pwmClawValue     = mapFloat(receivedAngles.clawTheta    , 0, 180, SERVOMIN2, SERVOMAX2);
            
            smoothMove(SER0, currentBasePwm    , pwmBaseValue)    ;
            smoothMove(SER1, currentShoulderPwm, pwmShoulderValue);
            smoothMove(SER2, currentElbowPwm   , pwmElbowValue)   ;
            smoothMove(SER3, currentRollPwm    , pwmRollValue)    ;
            smoothMove(SER4, currentPitchPwm   , pwmPitchValue)   ;
            smoothMove(SER5, currentClawPwm    , pwmClawValue)    ;

            currentElbowPwm    = pwmElbowValue   ;
            currentShoulderPwm = pwmShoulderValue;
            currentBasePwm     = pwmBaseValue    ;
            currentRollPwm     = pwmRollValue    ;
            currentPitchPwm    = pwmPitchValue   ;
            currentClawPwm     = pwmClawValue    ;
        }
    }
}



// // Function to enqueue servo movement
// void smoothMove(int servo, int currentValue, int targetValue) {
//     struct servoData moveInstruction = {servo, currentValue, targetValue};
//     xQueueSend(servoMoveQueue, &moveInstruction, portMAX_DELAY);
// }

// void moveAssemblyJstTask(void *pvParameters)
// {
//     struct angles receivedAngles;
//     while (1)
//     {
//         if (xQueueReceive(
//                 moveAssemblyJstQueue,
//                 &receivedAngles,
//                 portMAX_DELAY) == pdTRUE)
//         {
    
//             int pwmShoulderValue = map(receivedAngles.shoulderTheta, 0, 270, SERVOMIN1, SERVOMAX1);
//             int pwmElbowValue    = map(receivedAngles.elbowTheta   , 0, 180, SERVOMIN1, SERVOMAX1);
//             int pwmBaseValue     = map(receivedAngles.baseTheta    , 0, 180, SERVOMIN1, SERVOMAX1);
//             int pwmRollValue     = map(receivedAngles.rollTheta    , 0, 180, SERVOMIN2, SERVOMAX2);             
//             int pwmPitchValue    = map(receivedAngles.pitchTheta   , 0, 180, SERVOMIN2, SERVOMAX2);

  
//             smoothMove(SER1, currentShoulderPwm, pwmShoulderValue);
//             smoothMove(SER2, currentElbowPwm   , pwmElbowValue)   ;
//             smoothMove(SER0, currentBasePwm    , pwmBaseValue)    ;
//             smoothMove(SER3, currentRollPwm    , pwmRollValue)    ;
//             smoothMove(SER4, currentPitchPwm   , pwmPitchValue)   ;
            
//             currentElbowPwm    = pwmElbowValue   ;
//             currentShoulderPwm = pwmShoulderValue;
//             currentBasePwm     = pwmBaseValue    ;
//             currentRollPwm     = pwmRollValue    ;
//             currentPitchPwm    = pwmPitchValue   ;
//         }
//         else{
//           vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }
// }


void setup() {
  Serial.begin(115200);

  // Initialize Servo Driver
  driver.begin();
  driver.setPWMFreq(50);
    
  Wire.begin();
   
  // Start Wi-Fi in Access Point mode
  WiFi.softAP(ssid, password, channel);
  Serial.println("Wi-Fi Access Point started");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
  delay (300);  
  // Start the server

    // WiFi.mode(WIFI_STA);
    // if (esp_now_init () != ESP_OK) {
    // Serial.print ("\nError initializing ESP-NOW!");
    // return;
    // } 
    // esp_now_register_recv_cb (esp_now_recv_cb_t(onDataRecv));
    
    // Initialize Queues
    moveAssemblyQueue    = xQueueCreate(50, sizeof(struct angles));
    moveAssemblyJstQueue = xQueueCreate(50, sizeof(struct angles));
    moveAssemblyWCQueue  = xQueueCreate(1000, sizeof(struct dataPacket));
    servoMoveQueue       = xQueueCreate(100, sizeof(struct servoData)) ;
    recordDataQueue      = xQueueCreate(100, sizeof(struct angles));
    
    if (moveAssemblyWCQueue == NULL || servoMoveQueue == NULL) {
      Serial.println("Failed to create queue!");
      while (1); // Halt execution  
    }
   
    // Initialize Tasks
    
    //task for wifi communications
    if(xTaskCreatePinnedToCore(
      handleWifiTask,
      "handleWifi"  ,
       90000         ,
       NULL         ,
       0            ,
       &taskHandleWifi,
       0
    )!= pdPASS)
    {Serial.println("Tskwificreation failed!");}

    //Task to calculate angles
    if(xTaskCreatePinnedToCore(
      calculateAnglesTask ,
     "CalculateAngles"    ,
      4096                ,
      NULL                ,
      2                  , //priority @ 2
      &taskCalculateAngles, 
      1                     //pinned @ core 1
      ) != pdPASS)
      Serial.println("TaskClacualteAngles creation failed!");
    
    if(xTaskCreatePinnedToCore(
      moveAssemblyJstTask  ,
      "MoveAssemblyJst"    ,
       4096                , 
       NULL                ,
       3                   , //priority @ 2
       &taskMoveAssemblyJst,
       1                     //pinned at core 1
      ) != pdPASS)
      {Serial.println("TaskmoveAssembly creation failed!");}
      
    // if(xTaskCreatePinnedToCore(
    //   takeSerialInputTask , 
    //   "takeSerialInput"   ,
    //   2000                ,
    //   NULL                ,
    //   0                   ,//priority at 0
    //   &taskTakeSerialInput,
    //   0                    //runs on core 0
    // ) != pdPASS)
    // {Serial.println("TaskTakeSerialinput creation failed!");}
   
  //*******Initialize Servos to Neutral Positions********

    //initializing baseServo @90 degrees
    currentBasePwm = map(90, 0, 180, SERVOMIN1, SERVOMAX1);
    driver.setPWM(SER0, 0, currentBasePwm);

     //initializing shoulderServo @105 degrees
     currentShoulderPwm = map(105, 0, 270, SERVOMIN1, SERVOMAX1);
     driver.setPWM(SER1, 0, currentShoulderPwm);
    
    //iniitalizing elbowServo @90 degrees
    currentElbowPwm = map(90, 0, 180, SERVOMIN1, SERVOMAX1);
    driver.setPWM(SER2, 0, currentElbowPwm);
    
    //initializing roll servo @
    currentRollPwm = map(0, 0, 180, SERVOMIN2, SERVOMAX2);
    driver.setPWM(SER3, 0, currentRollPwm);

    //initializing pitchServo @90 degrees
    currentPitchPwm = map(90, 0, 180,SERVOMIN2, SERVOMAX2);
    driver.setPWM(SER4, 0, currentPitchPwm);
    
    //initializing clawServo @80 degrees
    currentClawPwm = map(68, 0 , 180, SERVOMIN2,SERVOMAX2);
    driver.setPWM(SER5, 0, currentClawPwm);

}

void loop() {
   vTaskDelete(NULL);
}


// void playRecordingTask(void *pvParameters)
// { struct node *temp = rootNode;
//   while(1)
//   {
//     if(playRecording)
//     {
//       if(xSemaphoreTake(moveBodyArmAssemblyMutex,portMAX_DELAY) == pdTRUE)
//       {
//         while(temp != NULL)
//         {       
//             int pwmShoulderValue = mapFloat(temp -> anglesRecorded -> shoulderTheta, 0, 270, SERVOMIND, SERVOMAXD);
//             int pwmElbowValue    = mapFloat(temp -> anglesRecorded -> elbowTheta   , 0, 180, SERVOMIND, SERVOMAXD);
//             int pwmBaseValue     = mapFloat(temp -> anglesRecorded -> baseTheta    , 0, 180, SERVOMIN1, SERVOMAX1);
//             int pwmRollValue     = mapFloat(temp -> anglesRecorded -> rollTheta    , 0, 180, SERVOMIN2, SERVOMAX2);             
//             int pwmPitchValue    = mapFloat(temp -> anglesRecorded -> pitchTheta   , 0, 180, SERVOMIN2, SERVOMAX2);
//             int pwmClawValue     = mapFloat(temp -> anglesRecorded -> clawTheta    , 0, 180, SERVOMIN2, SERVOMAX2);
            
//             Serial.println("PLAYINGGGGGGGGGGGGG");

//             smoothMove(SER0, currentBasePwm    , pwmBaseValue)    ;
//             smoothMove(SER1, currentShoulderPwm, pwmShoulderValue);
//             smoothMove(SER2, currentElbowPwm   , pwmElbowValue)   ;
//             smoothMove(SER3, currentRollPwm    , pwmRollValue)    ;
//             smoothMove(SER4, currentPitchPwm   , pwmPitchValue)   ;
//             smoothMove(SER5, currentClawPwm    , pwmClawValue)    ;

//             currentElbowPwm    = pwmElbowValue   ;
//             currentShoulderPwm = pwmShoulderValue;
//             currentBasePwm     = pwmBaseValue    ;
//             currentRollPwm     = pwmRollValue    ;
//             currentPitchPwm    = pwmPitchValue   ;
//             currentClawPwm     = pwmClawValue    ;

//             temp = temp->nextNode;
//         }
//         if(temp == NULL)
//         {
//          playRecording = false;
//         }
//       }
//       else
//       {
//       Serial.println("FAILED TO acquire movebodyarmAssemblymutex!");
//       }   
//     }
//     else
//     {//check for if playRecording is true every 100 milliseconds
//      vTaskDelay(pdMS_TO_TICKS(100));
//     }  
//   }
// }


 
    // if(xTaskCreatePinnedToCore(
    //   recordDataTask  ,
    //   "RecordDataTask",
    //   2000            ,
    //   NULL            ,
    //   2               ,  //priority @ 2
    //   &taskRecordData ,
    //   0                  // pinned @ core 0
    // ) != pdPASS)
    // {Serial.println("RecrodDATAtask creation failed!");}

     // if(xTaskCreatePinnedToCore(
    //   playRecordingTask  ,
    //   "PlayRecordingTask",
    //   5000               ,
    //   NULL               ,
    //   4                  , //priority @ 4
    //   &taskPlayRecording ,
    //   0                   //pinned at core 1
    // ) != pdPASS)
    // {Serial.println("PlayRecordingTaskCreation Failed!");}
  //   servoUpdateTimer = xTimerCreate(
  //   "ServoUpdateTimer",        // Timer name
  //   pdMS_TO_TICKS(5),          // Period (100 ms)
  //   pdTRUE,                    // Auto-reload
  //   NULL,                      // Timer ID
  //   servoUpdateCallback        // Callback function
  //  );

  //  if (servoUpdateTimer == NULL) {
  //   Serial.println("Timer creation failed!");
  //   while(1); // Stay in a loop to prevent further execution
  // }

  //   // Start the timer
  //   if (xTimerStart(servoUpdateTimer, 0) != pdPASS) {
  //     Serial.println("Failed to start servoUpdateTimer!");
  //   }
  //