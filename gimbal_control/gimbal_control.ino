/* Rocket Gimbal Stabilization
 *  
 *  
 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <ESP32Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

//SD Card headers
#define FS_NO_GLOBALS
#include "FS.h"
#include "SD.h"
#include "SPI.h"

boolean launch = 1;

double mpu6050_offsets[6] = {-4318, -6790, 9830,  203, -36, 46};

double k_d = 0;
double k_p = 0.25;

double error_past;
double error;
double d_error;
double gamma_temp;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// 16 servo objects can be created on the ESP32
Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
double servo_conv = 1.0/0.55;

boolean calibration;
int zero_a;
int zero_b;
double a;
double b;
double c;

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin1 = 13;
int servoPin2 = 12;

int LEDpin = 33;
int button = 0;
int LED = 5;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
double euler[3];        // [alpha, beta, gamma]    Euler angle container (Z-X'-Z'
Quaternion q;           // [w, x, y, z]         quaternion container

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

File datafile;
char * data_file_path;
boolean file_closed = false;
char * data_array = (char *) malloc(57);
unsigned long timestamp;


//Function Declaration
void dmpDataReady() {
    mpuInterrupt = true;
}

void createDir(fs::FS &fs, const char * path){
  fs.mkdir(path);
}


void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(5, 1);
  pinMode(0, INPUT);
  
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin,HIGH);

  // Servo setup
  // Default min/max of 1000us and 2000us (different servos may require different min/max settings)
  servo1.setPeriodHertz(50);    // standard 50 hz servo
  servo2.setPeriodHertz(50);    // standard 50 hz servo
  servo1.attach(servoPin1, 1000, 2000); // attaches the servo on pin 18 to the servo object
  servo2.attach(servoPin2, 1000, 2000); // attaches the servo on pin 18 to the servo object

  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(21 , 22, 400000);
        Serial.print("testasdfasdfasdf");
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
        Serial.print("2ndtestasdfasdfasdf");
    #endif

  
  // Initialize serial communication
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
  if (launch){
    // initialize device  
    zero_a = 67;
    zero_b = 80;
    servo1.write(zero_a);
    servo2.write(zero_b);
  }
  else {
    // initialize device
    calibration = true;
    Serial.println(F("\nInput gimbal motor 1 (+x-axis) initial position: "));
    while(calibration){
      if (Serial.available()) {
        int reading = Serial.parseInt();
          if (reading<0) {
            calibration = false;
          }
          else if (reading==0) {
          }
          else {
            zero_a = reading;
            servo1.write(reading);
          }
      }
      delay(15);
    }
    calibration = true;
  
    Serial.println(F("\nInput gimbal motor 2 (+y-axis) initial position: "));
    while(calibration){
      if (Serial.available()) {
        int reading = Serial.parseInt();
          if (reading<0) {
            calibration = false;
          }
          else if (reading==0) {
          }
          else {
            zero_b = reading;
            servo2.write(reading);
          }
      }
      delay(15);
    }
  }

  if(!launch){
    Serial.println(F("Initializing I2C devices..."));
  }
  mpu.initialize();
   // verify connection
  if(!launch){
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  }
  // wait for ready
  if(!launch){
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  }
  else{
    while (Serial.available() && Serial.read()); // empty buffer
  }
  // load and configure the DMP
  if(!launch){
    Serial.println(F("Initializing DMP..."));
  }
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(mpu6050_offsets[0]);
  mpu.setYAccelOffset(mpu6050_offsets[1]);
  mpu.setZAccelOffset(mpu6050_offsets[2]);
  mpu.setXGyroOffset(mpu6050_offsets[3]);
  mpu.setYGyroOffset(mpu6050_offsets[4]);
  mpu.setZGyroOffset(mpu6050_offsets[5]);
   
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
     
    if(!launch){
      Serial.println(F("Enabling DMP..."));
    }
    mpu.setDMPEnabled(true);
          
    //Setup interrupt for MPU6050
    if(!launch){
      Serial.println(F("Enabling interrupt detection (ESP32 pin 32)..."));
    }
    pinMode(32, INPUT_PULLUP);
    attachInterrupt(32, dmpDataReady, RISING);
      
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    if(!launch){
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
    }
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    if(!launch){
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
    digitalWrite(LEDpin,LOW);
  }

  if(!SD.begin()){
      if(!launch){
        Serial.println("Card Mount Failed");
      }
      digitalWrite(LEDpin,LOW);
      return;
  }
  File root = SD.open("/");
  File file = root.openNextFile();
  if(!file){
    if(!launch){
      Serial.println("Failed to open directory");
    }
    digitalWrite(LEDpin,LOW);
  }
  int launch_number = 1;
  while(file) {
    if (strstr(file.name(), "launch") != NULL) {
      if(!launch){
        Serial.print("Made it: ");
        Serial.println(file.name()[strcspn(file.name(), "0123456789")]-'0');
      }
      launch_number = file.name()[strcspn(file.name(), "0123456789")]-'0'+1;
    }
    file = root.openNextFile();
  }
  char * launch_dir = (char *) malloc(1 + strlen("/launch") + 1);
  strcpy(launch_dir, "/launch");
  launch_dir[strlen("/launch")] = launch_number + '0';
  launch_dir[strlen("/launch")+1] = '\0';
  data_file_path = (char *) malloc(1 + strlen(launch_dir) + strlen("/data.csv"));
  strcpy(data_file_path, launch_dir);
  strcat(data_file_path, "/data.csv");
  strcat(data_file_path, "\0"); //untested
  if(!launch){
    Serial.print("Launch Dir: ");
    Serial.println(data_file_path);
  }
  createDir(SD, launch_dir);
  datafile = SD.open(data_file_path, FILE_WRITE);
  datafile.print("time(ms),q0,q1,q2,q3,a_x,a_y,a_z\n");
}



void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
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
        if(!launch){
          Serial.println(F("FIFO overflow!"));
        }

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // Get quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);


        //Calculate Euler angles from rotation matrix elements
        euler[0] = atan2(2*(q.w*q.y+q.x*q.z), 
                      2*(q.w*q.x-q.y*q.z));     //arctan2(R_13, -R_23)
        euler[2] = atan2(2*(q.x*q.z-q.w*q.y), 
                      2*(q.w*q.x+q.y*q.z));     //arctan2(R_31, R_32)
        euler[1] = atan2(2*(q.w*q.x+q.y*q.z)/cos(euler[2]), 
                      pow(q.w, 2.0)-pow(q.x, 2.0)-pow(q.y, 2.0)+pow(q.z, 2.0));   //arctan2(R_32/cos(gamma), R_33)
/*
        if(!launch){
          Serial.print("Euler Angles: \t");
          Serial.print(euler[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(euler[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(euler[2] * 180/M_PI);
        }
*/
    }
    gamma_temp = euler[2];
    error = euler[1]*180/M_PI;      //Some function of beta
    
    d_error = error - error_past; //TODO change to abs()
    error_past = error;

    c = max(cos(11.0*M_PI/180.0), cos(0.75*error*M_PI/180.0)); // for launch use P= 0.5 maybe

//    c = max( (double)exp(-0.05*(k_p*abs(error) + k_d*d_error)), (double)0.9);
    a = (double)asin((double)pow(((1.0-pow(c,2.0))/(1.0+pow((c*tan(gamma_temp)),2.0))),(0.5)));
    b = ((double)((double)acos((double)c/(double)cos((double)a))));
    if (b<0){
      b = -b;
    }
    if (a<0){
      a = -a;
    }
        
//    double test1 = (double)acos((double)c/(double)cos((double)a));
//    double test3 = cos((double)a);
//    double test2 = (double)c;  

//TODO FIX SINCE 0<BETA<PI

    if(gamma_temp > 0){
        b = -b;
    }
    if(error < 0){
        a = -a;
        b = -b;
    }
    if (gamma_temp<0){
      gamma_temp = -gamma_temp;
    }
    if((double)gamma_temp >= ((double)M_PI/2.0)){
        a = -a;
    }
/*    
    Serial.print("\t");
    Serial.print(c);
    Serial.print("\t");
    Serial.print(a*180/M_PI);
    Serial.print("\t");
    Serial.println(b*180/M_PI);
*/
    //Write correction to servo()
    servo1.write(zero_a + servo_conv*1.0856*a*180.0/M_PI);
    servo2.write(zero_b + servo_conv*2.1213*b*180.0/M_PI);

    timestamp = millis();
    sprintf(data_array, "%6lu", timestamp);
    data_array[6] = ',';
    dtostrf(q.w, 6, 4, data_array+7);
    data_array[13] = ',';
    dtostrf(q.x, 6, 4, data_array+14);    
    data_array[20] = ',';
    dtostrf(q.y, 6, 4, data_array+21);
    data_array[27] = ',';
    dtostrf(q.z, 6, 4, data_array+28);
    data_array[34] = ',';
    sprintf(data_array+35, "%6d", aaReal.x);
    data_array[41] = ',';
    sprintf(data_array+42, "%6d", aaReal.y);
    data_array[48] = ',';
    sprintf(data_array+49, "%6d", aaReal.z);
    data_array[55] = '\n';    
    data_array[56] = '\0';    
    datafile.print(data_array);

    if(digitalRead(button) == LOW){
      if (!file_closed){
        file_closed = true;
        datafile.close();
      }
      digitalWrite(LED,0);
    }

}
