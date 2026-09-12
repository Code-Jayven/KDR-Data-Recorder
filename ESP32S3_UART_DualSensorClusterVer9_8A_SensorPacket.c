//assisted using ChatGPT, revisited using Deepseek (2026)
//JAYVEN A AGUIRRE
//VER 0.98A
//Testing for 6 pin UART Connections with SBC, 2 sensors for 1 intermediate ESp32S3 chip
//USes HALF Duplex modes, should be synchronized, usable with half duplex modules like the MAX series 485 (but it has to be 3.3V so use another chip).
//This version is a hardware version for R485 half duplex hardware

//VER0.96A
//changed pinouts because some communcation lines were not usable, good thing this iss till in prototyping phase
//VER0.96.B
//works for MAX485 but voltage compatability is an issue, we need to use 3.3v if we want to use this version
//VER0.97.A
//version removes 485 hardware and sets to full duplex, null modern approach of wiring
//VER0.98.A        5/22/26
//Revamps the RX and TX portions of the code, more control AND has motor control pins(PWM capability), other large structure changes that mainly revolve around commms, total Comms revamp

#include <Arduino.h> //Base Arduino
#include <Wire.h> //I2C
#include <DFRobot_BMI160.h> //BMI160 Interface
#include <math.h> //math tools for preprocess
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>
#include <stdbool.h> //boolean here helps save states with less memory cost (and helps w confusion)
//We only really need this to disable WIFI and BLUETOOTH, this MCU project only needs wired connections
#include <WiFi.h>
#include <esp_bt.h>
#include <string.h> //mem set clearing

//ESP32S3 Hadware Allocation pints
#define LED_PIN 48 // GPIO pin connected to the LED
#define DL1 11 //Direct GPIO Line pin 1
#define DL2 12 //Direct GPIO Line pin 2
#define I2CSDL 9 //SCL Connection to BMI's (if needed theyll be here for documentaiton)
#define I2CSDA 8 //SCA connection to BMI160's
//#define MAX_DE_RE 10 //47 for final product  //Indicator pin that tells MCU that main comms RS chip is sending recieving.
// uart

HardwareSerial UART(2);   // Use UART2
//#define RS485_TX_PIN 1   // Connect to MAX485 DI
//#define RS485_RX_PIN 2   // Connect to MAX485 RO

#define AI1 4 //Analog input pin 1, goes to 4 
#define AI2 5
#define AI3 6
#define AI4 7 //Last Analog input pin
#define SENSORID 0x05 //this is for sensorpack
#define SENSORCHECK 0x5B //check for this pack, if it is off by even 1 the whole packet is disregarded!
#define VERSION "0.98A"
//Motor and GPIO Operation pins, good for updates but we need PWMs!
#define MOTO1L 17
#define MOTO2L 16
#define MOTO1R 15
#define MOTO2R 14
//additional GPIOs here
//#define GO1Y 
//#define GO2Y 
//#define GO3Y 
//#define GO4Y
//Upper BMI160 limb (refer to as 1)
DFRobot_BMI160 bmi160_upper;
//Lower BMI160 limb (refer to as 2)
DFRobot_BMI160 bmi160_lower;


SemaphoreHandle_t dataMutex;
TaskHandle_t sensor1TaskHandle, sensor2TaskHandle, TX_TaskHandle , RX_TaskHandle;

const int8_t SAO_V = 0x69; //2 I2C's for BMI
const int8_t SAO_G = 0x68; //2 I2C's for BMI
const float dtt = 0.005f; //5ms reading, change me later with benchmarking
//ints and global memory
int MotorUpper = 0; //initial state of motors lower speed, 0 means HALT
int MotorLower = 0; //Upper = "1", Lower = "2" (via ordering!)
bool Stream = false; //state here tells us if we need to rapid fire pacekts, otherwise it skips the constructor!
bool packhalf1 = false; //we use these to send packets when new data is READY, both must be TRUE
bool packhalf2 = false;
//ack for TX
bool PackToggAck = false; //0x01 acknowledgement
bool PackStopAck = false; //0x01 acknowledgement
bool RollCallAck = false; //0x03 acknowledgement,tells us IF we need to send a roll call packet back
bool MotorDriveAck = false; //0x04 ack
bool FreshERROR = false; //0x06 ack, tells system to send an error packet, will not send error packets until it is sent!
bool packetAnnounce = true; //debugger function that just propounces how many sensor packets we spit out
long int packetOUT = 0; //debugger function variable that tells us HOW MANY pacekts we sent during a stream!

//UART Items
int RX_index = 0;
uint8_t RX_Buffer[4];


//Collected IMU Data for magdwick preprocess,
struct IMUData{
  float ax, ay, az;  // Accelerometer in g's
  float gx, gy, gz;  // Gyroscope in degrees/second
  float dt;          // Sampling period in seconds
};

struct Quaternion { //Squant Structure
  float q0;
  float q1;
  float q2;
  float q3;
};

// ----Modified data packet struct for 2 sensor MCU combo
typedef struct __attribute__((packed)) { //TOTAL PAYLOAD: 39 bytes
  uint8_t ID; //needs ID since we have more packets to worry bout
  Quaternion q1;          // first quaternion, upper segment
  Quaternion q2;          //second quaternion, lower segment
  uint8_t as1[2];       // analog sensor data, each being from 0-255, upper segment
  uint8_t as2[2];     //  analog sensor data lower segment
  uint8_t comment;
  uint8_t check;
} SensorData; //classic sensor pack, Runs as 0x05 in the system

SensorData sensorData; //0x05 sensor /2 IMU 4 AN/ Packet
/*
typedef struct __attribute__((packed)) { //TOTAL PAYLOAD: 67 bytes
  uint8_t ID; //needs ID since we have more packets to worry about, this unique packet is 0x08
  Quaternion u1;          // first quaternion, humerous (upper arm) segment ~16 Bytes via FLOATING Point vals
  Quaternion l2;         //second quaternion, forearm segment
  Quaternion h1;         //hand segemnt
  uint16_t ADS1[4];       // analog sensor data from the ADS1115 or ADS1015 series ADC extension, 16 bytes per unit (~8 bytes per 4 unit list)
  uint16_t ADS2[4];       // analog sensor data from second ADS chip
  uint8_t comment;
  uint8_t check;
} BigPackData; //unique to ID 0x08 , Mainly for arms or sensor density resizing
*/
//UART based packets for Recieve and Sending (acknowlegement packets, CMD packets accepted {CMD packet NOT inclduded in this list})
typedef struct __attribute__((packed)){ //small CMD packet, check is usually XOR but list says it can be diffrent (must match or its not considered)
  uint8_t ID; //unique ID
  uint8_t Check; //check-value, doesnt have to be XOR but prefered
}BytePack2;
//BytePack2 based messages:
BytePack2 ToggPack = {0x01,0xFE}; //sends back same pack as acknoledgement
BytePack2 StopPack = {0x02,0xFD}; //sends back same pack as acknowledgement
BytePack2 RollCallINPack = {0x03,0xFC}; 
BytePack2 RollCallOUTPack = {0x03,0xCC};
BytePack2 MotorAckPack = {0x04,0xFC};

typedef struct __attribute__((packed)){ //4 Byte Packet for Detailed control (think motors, GPIO combinations)
  uint8_t ID; //unique ID
  uint8_t byteU; //special byte that
  uint8_t byteL; //Identifies the motor it controls (1 or 2), if above 2, then other functionality is present (other motors, GPIO ect.)
  uint8_t Check; //check-value, doesnt have to be XOR but prefered
}BytePack4;
//note, these come unmade, packets, fill er up
BytePack4 MotorPack = {0x04,0,0x00,0xFB}; //Motor command, reciever can draw on this as a "last recieved cmd" memory, transmitter uses memcopy to make new alterations
BytePack4 ERRPack = {0x04,0,0x00,0xFC}; //ERROR pack, integer is type, byte is a specific detail, refer to above for usage

// ----- Madgwick Update -----
void MadgwickUpdate(Quaternion &q, const IMUData &imu) {
  float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
  float ax = imu.ax, ay = imu.ay, az = imu.az;
  float gx = imu.gx, gy = imu.gy, gz = imu.gz;
  float dt = imu.dt;

  // Parameters
  const float beta = 0.1f;  // filter gain (tune for your application)

  // Rate of change of quaternion from gyroscope
  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);


  // Normalize accelerometer
  float recipNorm = sqrtf(ax * ax + ay * ay + az * az);
  if (recipNorm == 0.0f) return;
  recipNorm = 1.0f / recipNorm;
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;


  // Auxiliary variables to avoid repeated arithmetic
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1;
  float _8q2 = 8.0f * q2;
  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  // Gradient descent corrective step
  float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
  float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

  recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  // Apply feedback step
  qDot1 -= beta * s0;
  qDot2 -= beta * s1;
  qDot3 -= beta * s2;
  qDot4 -= beta * s3;

  // Integrate rate of change of quaternion
  q0 += qDot1 * (1.0f / dt);
  q1 += qDot2 * (1.0f / dt);
  q2 += qDot3 * (1.0f / dt);
  q3 += qDot4 * (1.0f / dt);

  // Normalize quaternion
  recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //writ
  q.q0 = q0;
  q.q1 = q1;
  q.q2 = q2;
  q.q3 = q3;
  
  //all saved towards q buffer
}

bool readBMI160(DFRobot_BMI160 &sensor, float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  int16_t data[6];  // accel x,y,z then gyro x,y,z
  if (sensor.getAccelGyroData(data) != BMI160_OK) {
    return false;
  }
  // Convert LSB to physical units
  // For accelerometer: 16384 LSB/g (assuming ±2g range)
  ax = data[0] / 16384.0f;
  ay = data[1] / 16384.0f;
  az = data[2] / 16384.0f;
  // For gyroscope: 131 LSB/°/s (assuming ±250°/s)
  gx = data[3] / 131.0f;
  gy = data[4] / 131.0f;
  gz = data[5] / 131.0f;
  return true;
}
//Both Sensor tasks
void sensor1Task(void *pvParameters) {
  // ... I2C initialization code ...
  Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    if (packhalf1 = true) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }
    // Read BMI160 #1
    float ax, ay, az, gx, gy, gz;
    if (readBMI160(bmi160_upper, ax ,ay ,az, gx ,gy ,gz)){
      // Convert raw values to g and °/s (adjust scales to your settings)
      // Example: for ±2g accelerometer, LSB = 0.000061g
      //          for ±250°/s gyro, LSB = 0.00763 °/s


      
      IMUData imu = {ax, ay, az, gx, gy, gz, dtt};  // dt = 5 ms, dtt is global
      MadgwickUpdate(q, imu);   // q is updated in-place
    }
      //This Core's share of the Analog readings here. slots 1 and 2 are filled
      // Update shared data (protected by mutex)
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      sensorData.q1 = q;
      xSemaphoreGive(dataMutex);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
} //END of the sensor task

void sensor2Task(void *pvParameters) {
  // ... I2C initialization code ...

  Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    //it must have both GPIO pins high
    if (packhalf2 = true) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }
    // Read BMI160 #1
    float ax, ay, az, gx, gy, gz;
    if (readBMI160(bmi160_lower, ax ,ay ,az, gx ,gy ,gz)){
      // Convert raw values to g and °/s (adjust scales to your settings)
      // Example: for ±2g accelerometer, LSB = 0.000061g
      //          for ±250°/s gyro, LSB = 0.00763 °/s
      IMUData imu = {ax, ay, az, gx, gy, gz, dtt};  // dt = 5 ms, dtt is global
      MadgwickUpdate(q, imu);   // q is updated in-place
    }
      //This Core's share of the Analog readings here. slots 1 and 2 are filled
      // Update shared data (protected by mutex)
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      sensorData.q2 = q;
      xSemaphoreGive(dataMutex);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
} //END of the sensor task

uint8_t readAnalog(int pin1){ // This function reads One of the FOUR specified pins that must be read to pack the packet, both cores should run this at LEAST once
    // Read 12-bit ADC value (0-4095) [citation:4]
    uint16_t rawValue1 = analogRead(pin1) >> 4;
    // Scale to 8-bit (0-255) by dividing by 16 (right shift 4 bits)
    return rawValue1;
}

void Operation(uint8_t cmd_byte, uint8_t spec_byte1, uint8_t spec_byte2){ //funny little function that, given 2 bytes, changes MCU operation (pwm, gpio, UART ect)
  if(cmd_byte == 0x01){ //PACKET TOGGLE, this turns on the packet stream,
    Serial.println("Stream Active");
    Stream = true; //Global variable
  } //end of 1
  else if(cmd_byte == 0x02){ //PACKET STOP, this turns OFF the packet stream,
    Serial.println("Stream Ended");
    if(packetAnnounce == true){
      Serial.print("Packets Send > %d", packetOUT);
    }
    packetOUT = 0;
    Stream = false; //Global variable
  } //end of 2
  else if(cmd_byte == 0x03){ //Roll Call, immediately pins TX task to send a roll call
    Serial.println("Roll Call");
    RollCallAck = true;   
  } //end of 3
  else if(cmd_byte == 0x04){ //Motor Drive, sets the motor of the machines choice to specified values, spec byte and other portions modify the speed
    Serial.println("Updating MCU Operation");
    if(spec_byte1 == 0x01){ //Upper Motor Change
      // changes motor from recieved data
      //add PWM output augmentation here
    }
    if(spec_byte1 == 0x02){ //Lower Motor Change
       // changes motor from recieved data
      //add PWM output augmentation here
    }      
    //Send ack when u can
  }//end of 4

}//end of Operation func

void RX_Task(void *pvParameters){ //code here listens for byte, then does, stuff I guess, reciever port
  //UART.begin(115200,SERIAL_8N1,2,1);  // 115200 kbps
  Serial.println("RX Communication task started");
  while(1){  
    if (UART.available() == 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    uint8_t rec = UART.read(); //we need to read the command, state machine should be below
    Serial.printf(">> 0x%02X\n", rec);

    RX_Buffer[RX_index] = rec; //load the type into the array
    Serial.println(RX_index);       

    ///STATE MACHINE, works with what index it is in given the
    switch(RX_index){//state machine that can take 2 types of packets, an 2 byte (2 item), and a 7 byte (4 item)
      case 0:{ //start of the data       
      if(rec == 0x01|| rec == 0x02|| rec ==0x03 || rec ==0x04 || rec == 0x05 ){ //this portion should read a VALID ID, the proceeds to the switch case
          //should increment from 0 _> 1
          RX_index++;
          Serial.println("2BytePack");
        }//Valid command type recieved, move on
        break;
      } //end of 0
      case 1: { //reads the byte, checks the type and the associated type in the index
        if(RX_Buffer[0] == (0x01)){ //These types are 2 Byte COMMANDS, this here is check, validate before executing commands!
          //above checks type, below we check for checksums, if correct, then it executes the command, resets state machine
          if(rec == 0xFE){//checksum check for 0x01, valid means we run this
            memset(RX_Buffer, 0x00, 4); //should reset the list
            Operation(0x01,0); //changes operation
            RX_index = 0; //resets index, reseting the machine
          }else{
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets regardless
          } //bad check, resets with no action
        }
        if(RX_Buffer[0] == (0x02)){ //These types are 2 Byte COMMANDS, this here is check, validate before executing commands!
          //above checks type, below we check for checksums, if correct, then it executes the command, resets state machine
          if(rec == 0xFD){//checksum check for 0x02, valid means we run this
            Operation(0x02,0); //changes operation
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets index, reseting the machine
          }else{
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets regardless
          } //bad check, resets with no action
        }
        if(RX_Buffer[0] == (0x03)){ //These types are 2 Byte COMMANDS, this here is check, validate before executing commands!
          //above checks type, below we check for checksums, if correct, then it executes the command, resets state machine
          if(rec == 0xFC){//checksum check for 0x03, valid means we run this
            Operation(0x03,0); //changes operation
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets index, reseting the machine
          }else{
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets regardless
          } //bad check, resets with no action
        }//end of 2 bytes
        if(RX_Buffer[RX_index == 0] == (0x04)){ //only if we get a motor pack we extend this list
          RX_Buffer[RX_index] = rec;
          RX_index++;
        }//give that we have a new struct, we just save the data and go to case 3 
        break;
      }//end of 1
      case 2:{ //all bytes, BUT we can convert to normal numbers, special math, annoying math, but usable math, can be used!
        if(RX_Buffer[RX_index == 0] == (0x04)){ //moves on
          RX_Buffer[RX_index] = rec;
        }else{//bad check, resets with no action
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets regardless
          } 
          break;
      }//end of CASE 2
      case 3:{ //specifically checks for CHECKSUMS agian, mainly for motorized pins
          if(rec == 0xFB && RX_Buffer[0] == (0x04)){//checksum check for 0x01, valid means we run this
            Operation(0x04,RX_Buffer[1]); //changes operation
            delay(40); //tiny delay
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets index, reseting the machine
          }else{
            memset(RX_Buffer, 0x00, 4); //should reset the list
            RX_index = 0; //resets regardless
          } //bad check, resets with no action
        break;
      }


    }//end of the WHOLE SWITCH!!!

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}//END OF FUNCTION

void TX_Task(void *pvParameters) { //sends things based on importance, used to be commTask (all in 1 func)

  //removed 485 hardware DERE pin requirements here, full duplex ver
  // receive mode when low
  Serial.println("TX Communication task started");
  // Configure state pins as inputs
  pinMode(DL1, INPUT_PULLUP);
  pinMode(DL2, INPUT_PULLUP);

  // Buffer for outgoing packet (1+16+16+4+1+1 = 39 bytes) 
  uint8_t packet[39];
  //Other pacekts are made
  while (1) { //This goes through the packet list, prioritizes based on which 1 of these r true first
      //PACKET is highest prios send, other items send whenever (0x05)
      if (Stream == true && packhalf1 == true && packhalf2 == true) {  // packet toggle is true, this should activate when its time to send fresh packets (especially when both halfs are FRESH)
        // Read analog sensors
        uint8_t analogVals[4]; 
        analogVals[0] = readAnalog(AI1);
        analogVals[1] = readAnalog(AI2);
        analogVals[2] = readAnalog(AI3);
        analogVals[3] = readAnalog(AI4);

        // Read state pins to form comment byte (bits 0 and 1)
        uint8_t comment = 0x00;
        // Build packet with mutex protection
        xSemaphoreTake(dataMutex, portMAX_DELAY); 
        //ID must be copied
        packet[0] = SENSORID;
        // Copy q1 (16 bytes)
        memcpy(packet + 1, &sensorData.q1, 16);
        // Copy q2 (next 16 bytes)
        memcpy(packet + 17, &sensorData.q2, 16);
        // Copy analog values (4 bytes)
        memcpy(packet + 33, analogVals, 4);
        // Copy comment byte (1 byte)
        packet[37] = comment;
        //create check
        packet[38] = SENSORCHECK;
        xSemaphoreGive(dataMutex);

        // Transmit
        // enable transmitter from SBC side, when it detects it sends a packet everytime
        UART.write(packet, 39);
        UART.flush();  // wait for completion
        //done, change the conditions so we dont send the same packet agian, allows other cores to craft fresh and new packets
        packhalf1 == false; //new data needed
        packhalf2 == false; //new data needed!
        packetOUT++;
    } //End of packet send
    else if (RollCallAck == true){ // Rolecall request, no response mean the MCU is busted OR not responsive,sends back acknoledgement (0x03)
      Serial.println("Roll call detected, responding...");
      UART.write((uint8_t*)&RollCallOUTPack, 2); //ends the premade struct, no main changes really
      UART.flush();
      Serial.println("Response sent"); 
      RollCallAck = false; //done, dont send twice
      vTaskDelay(pdMS_TO_TICKS(1));  // yield
    }//0x03 ended, shouldve only given 2 bytes
    else if (PackToggAck == true){ // Packet ON ack, 0x01
      Serial.println("Ack ON Packet Stream");
      UART.write((uint8_t*)&ToggPack, 2); //ends the premade struct, no main changes really
      UART.flush();
      Serial.println("Response sent"); 
      PackToggAck = false; //done, dont send twice
      vTaskDelay(pdMS_TO_TICKS(1));  // yield
    }//0x01 ended, shouldve only given 1 byte   
    else if (PackStopAck == true){ // Packet OFF ack, 0x02
      Serial.println("Ack OFF Packet Stream");
      UART.write((uint8_t*)&StopPack, 2); //ends the premade struct, no main changes really
      UART.flush();
      Serial.println("Response sent"); 
      PackStopAck = false; //done, dont send twice
      vTaskDelay(pdMS_TO_TICKS(1));  // yield
    }//0x02 ended, shouldve only given 1 byte   
    else if (MotorDriveAck == true){ // Packet OFF ack, 0x04
      Serial.println("Ack OFF Packet Stream");
      UART.write((uint8_t*)&MotorAckPack, 2); //ends the premade struct, no main changes really
      UART.flush();
      Serial.println("Response sent"); 
      MotorDriveAck = false; //done, dont send twice
      vTaskDelay(pdMS_TO_TICKS(1));  // yield
    }//0x04 ended, shouldve only given 1 byte
    else if (FreshERROR == true){ // Packet OFF ack, 0x04
      Serial.println("Ack OFF Packet Stream");
      UART.write((uint8_t*)&ERRPack, 4); //ends the premade struct, no main changes really
      UART.flush();
      Serial.println("Response sent"); 
      FreshERROR = false; //done, dont send twice
      vTaskDelay(pdMS_TO_TICKS(1));  // yield
    }//0x04 ended, shouldve only given 1 byte      
    vTaskDelay(pdMS_TO_TICKS(100));  
  }//end of while
//set this up to send packets via intervals WHEN state pins and transmit DE_RE pin are alined
}


//SETUP BEGIN!
void setup(){
  Serial.begin(115200);
  UART.begin(115200,SERIAL_8N1,2,1);  // 115200 kbps
  delay(50);
  Serial.println("Starting limb node...");
  Serial.println(VERSION);
  //START I2C
  Wire.begin(I2CSDA , I2CSDL,100000);
  //set up needed pins for any GPIO
  pinMode(DL1, INPUT_PULLUP);
  pinMode(DL2, INPUT_PULLUP);
  pinMode(AI1, INPUT_PULLUP);
  pinMode(AI2, INPUT_PULLUP);
  pinMode(AI3, INPUT_PULLUP); 
  pinMode(AI4, INPUT_PULLUP);
  //Turn off wireless nonsense
  WiFi.mode(WIFI_OFF);
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  btStop();
  // Initialize BMI160 sensors
  int init_upper_att = 0;
  int init_lower_att = 0;
  while (bmi160_upper.I2cInit(SAO_G) != BMI160_OK && init_upper_att < 5){
    Serial.println("Sensor 1 init failed, retrying...");
    delay(1000);
    init_upper_att++;
  }
  delay(100);
  if(init_upper_att >= 5){ //fail to init, will let u know
    Serial.println("Sensor 1 init failed attempts, proceeding");
  }else{
    Serial.println("Sensor 1 init detected! proceeding");
  }

  while (bmi160_lower.I2cInit(SAO_V) != BMI160_OK && init_lower_att < 5) {
    Serial.println("Sensor 2 init failed, retrying...");
    delay(1000);
    init_lower_att++;
  }
  if(init_lower_att >= 5){ //fail to init, will let u know
    Serial.println("Sensor 2 init failed attempts, proceeding");
  }else{ 
    Serial.println("Sensor 2 init detected! proceeding");
  }
  Serial.println("Both Sensor init processes done.");
  analogReadResolution(12);
  //Deubug/value cleanup set
  // Initialize quaternions to identity (1, 0, 0, 0),standard basic value
  sensorData.q1.q0 = 1.0f; sensorData.q1.q1 = 0.0f; sensorData.q1.q2 = 0.0f; sensorData.q1.q3 = 0.0f;
  sensorData.q2.q0 = 1.0f; sensorData.q2.q1 = 0.0f; sensorData.q2.q2 = 0.0f; sensorData.q2.q3 = 0.0f;
  ////////////////////////////////////////
  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  // Create tasks pinned to cores
  xTaskCreatePinnedToCore(sensor1Task, "Sensor1", 2048, NULL, 1, &sensor1TaskHandle, 0);
  delay(100);
  xTaskCreatePinnedToCore(sensor2Task, "Sensor2", 2048, NULL, 1, &sensor2TaskHandle, 1);
  delay(200);
  xTaskCreatePinnedToCore(TX_Task, "CommTX", 2048, NULL, 2, &TX_TaskHandle, 1); // higher priority for comm
  delay(200);
  xTaskCreatePinnedToCore(RX_Task, "CommRX", 4096, NULL, 2, &RX_TaskHandle, 1); // higher priority for comm
} //END OF SETUP

/////MAIN LOOP       ////////////////
void loop(){  
   vTaskDelay(portMAX_DELAY);
}//END OF LOOP