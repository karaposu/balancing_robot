/**********Bu Kod el emegi göz nurudur.********/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <TimerOne.h>
MPU6050 mpu;

        
// uncomment "Bluetooth" if you want to send gyro values over bluetooth
#define Bluetooth

// uncomment "robot" if you want to see the actual pid values 
//#define ROBOT


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


//---------------------------------------------------TB6612 VARIABLES  ⇓⇓⇓⇓⇓⇓⇓⇓⇓
int STBY = 9; //standby
//Motor A
int PWMA = 6; //Speed control
int AIN1 = 8; //Direction
int AIN2 = 7; //Direction

//Motor B
int PWMB = 11; //Speed control
int BIN1 = 10; //Direction
int BIN2 = 12; //Direction


//---------------------------------------------------MPU6050 VARIABLES  ⇓⇓⇓⇓⇓⇓⇓⇓⇓
  int sira=0, tasima=0;
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool blinkState = false;
float ax, ay, az;
float gx, gy, gz;


//---------------------------------------------BLUETOOTH VARIABLES  ⇓⇓⇓⇓⇓⇓⇓⇓⇓
   int digit_1000,digit_100,digit_10,digit_1;
   int sign_of_number;
const int packet_size=5;
uint8_t sign_byte=0;
float fraction_packet[8];
uint8_t sign_packet[8];
float  packet2send[packet_size];
float  example_packet_253[3]={-192.52 ,250.45 , 255 };
float  example_packet_254[3]={-10.45 ,-15.255 , 200 };
uint8_t  packet_p253_edited[packet_size];
//---------------------------------------------PID VARIABLES   ⇓⇓⇓⇓⇓⇓⇓⇓⇓

int pwm_cikis=0;;  
float tune =0;float tune2 =0;float tune3 =0;
float raw_pid=0;
uint8_t first_time=0;;
float iTerm = 0;
float ki, kp, kd;
float dt;
float angle_error_old=0;

void parse_to_digits(int number);    
void dmpDataReady() {
    mpuInterrupt = true;
}

//---------------------------------------------------Modifying variables ⇓⇓⇓⇓⇓⇓⇓⇓⇓
 
int        aci_eksi_sinir=-4,        aci_arti_sinir=4;  //min negatitive and positive angle borders for balance robot
int        aci_max_eksi_sinir=-40,   aci_max_arti_sinir=40; //max negatitive and positive angle borders for balance robot
int        deadband_pwm_n=-25,        deadband_pwm_p=25;  //to remove deadband in pwm signal and motor connection

float      angle_offset=9.7 ;





void setup() {
 
settings();

 Timer1.initialize(100000); // set timer  for every ... ms and calculate pid again in each period
 dt=100;
 Timer1.attachInterrupt( timerIsr ); // attach the service routine here

}

void loop() {
//unsigned long start = micros();

if(first_time==0){delay(2000); first_time=1;}

  //potantiometer reading for p i d tunning
double sensorValue = analogRead(A0);//p
double sensorValue2 = analogRead(A1);//i
double sensorValue3 = analogRead(A2);//d
tune=  sensorValue*6/1023.0;   // changes tune value between 0 and 6
tune2=  sensorValue2*50/1023.0;   // changes tune value between 0 and 6
tune3=  sensorValue3*100/1023.0;   //changes tune value between 0 and 6

kp=tune; ki=tune2; kd=tune3; 

dmp(); // calculates gx value via digital motion processor in mpu6050


 gx=ypr[2]* 180/M_PI;  // pitch
 gx=gx-angle_offset;

packet2send[0]=gx;
packet2send[1]=kp;
packet2send[2]=raw_pid;



   #ifdef Bluetooth
   // Serial.println(gx);
    // bluetooth(gx,kp,raw_pid);
//    
//        Serial.println();  
//       Serial.println(example_packet_253[0]);    //sign byte
       Serial.println(gx);  
   //      protocol_253(packet2send);
       
     
     
   #endif 

pid_control();



//unsigned long end = micros();
//unsigned long delta = end - start;
//Serial.println(delta);
}




void timerIsr(){


pid_update();


//  #ifdef Bluetooth
//   // Serial.println( gx,kp);
//     bluetooth(gx,kp,raw_pid);
//    // delay(15);
//    #endif 

}



//
//
//void execution_time()
//{
//
// unsigned long time =millis();
// for(int i=0;i<1000;++i); // an empty loop
// Serial.println(millis()-time);
//
// time =millis();
// for(int i=0;i<1000;++i) {
//   //your function here
// }
// Serial.println(millis()-time);
//}
//
//
//









