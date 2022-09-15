// Libraries:
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>


// Pins: 
//// PWM motor pins:
#define LMotPWMPin  9 
#define RMotPWMPin  6

#define LMot2PWMPin  10
#define RMot2PWMPin  5

//// Dir motor pins:
#define LMotDIRPin  8 
#define RMotDIRPin  7 

#define LMot2DIRPin  11 
#define RMot2DIRPin  4

//// Motor variables:
int LMotor = 1;
int RMotor = 0;

//// Encoder pins:
#define InterEncoderL 0 
#define LEncoderA 2
#define LEncoderB 12

#define InterEncoderR 1
#define REncoderA 3
#define REncoderB 13


// Motor Parameters:
float WheelSeparation = 0.3;
float WheelDiameter = 0.1;
int TPR = 768;      // Encoder ticks per rotation 
int AccParam = 3;   // Acceleration multiplier
double Prev_T;
double Prev_Error;
double Prev_Input;
double Accu_Error;
double pidterm;
int Kp = 0.8;       // PID proportional control gain.
int Kd = 1.3;       // PID Derivative control gain

// sudo vars: 
int OdomWait = 3;
int OdomCount = 0;
double WCS[2] = {0,0};

// ROS variables: 
ros::NodeHandle nh;
//// ROS publisher
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("nexus_odom", &odom_msg);
geometry_msgs::Twist debug_msg;
ros::Publisher Debug ("debug", &debug_msg);
//ROS subscriber

void messageCb( const geometry_msgs::Twist& CVel){
  //geometry_msgs::Twist twist = twist_msg;   
    double vel_x = CVel.linear.x;
    double vel_th = CVel.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    // Movement:
    // turning
    if(vel_x == 0){  
        right_vel = vel_th * WheelSeparation / 2.0;
        left_vel = (-1) * right_vel;
    }
    // forward / backward:
    else if(vel_th == 0){ 
        left_vel = right_vel = vel_x;
    }
    // moving doing arcs:
    else{ 
        left_vel = vel_x - vel_th * WheelSeparation / 2.0;
        right_vel = vel_x + vel_th * WheelSeparation / 2.0;
    }
    //write new command speeds to global vars 
    WCS[0] = left_vel;
    WCS[1] = right_vel;
}

ros::Subscriber<geometry_msgs::Twist> Sub("cmd_vel", &messageCb );

// Motor variables: 
#define CW   1
#define CCW  2
int inApin[4] = {LMotDIRPin, RMotDIRPin,LMot2DIRPin, RMot2DIRPin};     // INDIR: Clockwise input
int pwmpin[4] = {LMotPWMPin, RMotPWMPin,LMot2PWMPin, RMot2PWMPin};     // PWM input
int MotorNum[2] = {LMotor, RMotor};

//encoder variables:
Encoder LEncoder(LEncoderA, LEncoderB);
Encoder REncoder(REncoderA, REncoderB);

long EncoderVal[2] = {0,0};
double DDis[2] = {0,0};
long Time[2] = {0,0};

//debug variables:
double Vels[2] = {0,0};
int CVEL[2]= {0,0};
int Mspeeds[2] = {0,0};


// Main Program:
//// Setup Code: 
void setup()
{
    nh.getHardware()->setBaud(19200);
    nh.initNode();  
    nh.advertise(Pub);
    nh.advertise(Debug);
    nh.subscribe(Sub);

    nh.getParam("/serial_node/WheelSeparation", &WheelSeparation,1);
    nh.getParam("/serial_node/WheelDiameter", &WheelDiameter,1);
    nh.getParam("/serial_node/AccParam", &AccParam,1);
    
    // Pin mode for motors
    pinMode(LMotDIRPin,OUTPUT);
    pinMode(RMotDIRPin,OUTPUT);
    pinMode(LMot2DIRPin,OUTPUT); 
    pinMode(RMot2DIRPin,OUTPUT);
    

}

//// Main code:
void loop(){

    nh.spinOnce();
    
    //first couple of times dont publish odom
    if(OdomCount > OdomWait){
      odom_msg.linear.x = Vels[0];
      odom_msg.linear.y = Vels[1];
    Pub.publish(&odom_msg);
  }
  else{OdomCount++;}

//// Motor variables:
//int LMotor = 1;
//int RMotor = 0;

    // Assiging values to debugger message:
    debug_msg.linear.x = WCS[0];
    debug_msg.linear.y = Vels[0];
    debug_msg.linear.z = Mspeeds[0];
    debug_msg.angular.x = WCS[1];
    debug_msg.angular.y = Vels[1];
    debug_msg.angular.z= Mspeeds[1];
    
    // Publishing the debug message:
    Debug.publish(&debug_msg);

    MotorWrite(); //Takes WCS and corrects speed of motors with encoders    

    delay(3);
}


//// Encoder function code:
// Motor write speed variables(in motor units):
double MWS[2]= {0,0};

double CorrectedSpeed(int M, double CVel)
{
  //if fist time in program return 0 and init time vars
  if(Time[0]==0 && Time[1] == 0){
    Time[0] = millis();
    Time[1] = millis();
    return 0;
  }

  //read encoder ticks
  if(M == LMotor){
    EncoderVal[0] = LEncoder.read();
    LEncoder.write(0);
  }
  if(M == RMotor){
    EncoderVal[1] = REncoder.read();
    REncoder.write(0);
  }

  //differencial of time in seconds
  long T = millis();
  int DTime = T-Time[M];
  Time[M] = T;


  //diferential of distance in meters
  DDis[M] = TicksToMeters(EncoderVal[M]);
  
  //calculate short term measured velocity
  double EVel = (DDis[M]/DTime)*1000;
  
  //save to publish to /ard_odom
  Vels[M] = EVel;

  EVel = abs(EVel);
  CVel = abs(CVel);

   // Differential in calculation Time:
  unsigned long T_Now = millis();
  double Delta_T = double (T_Now - Prev_T);
  Prev_T = T_Now;
        
   // Propotional controller:
  double dif = EVel - CVel;

  // Differential controller:
  double Der_Error = (dif - Prev_Error) / Delta_T;
        
  // PID Term:
  double pidterm = (Kp * dif) + (Kd * Der_Error);   // PD controller
        
  // PWM output to motors
  if(MWS[M]<60 && MWS[M]>=0)
        {
          MWS[M]=MWS[M]-pidterm;
          }
  if(MWS[M]>60)
        {
          MWS[M]=59;
          }
  if(MWS[M]<0)
        {
          MWS[M]=0;
          }
  
  if(CVel == 0)
        {
          MWS[M] = 0;
          }

  //DEBUG
  CVEL[M] = MWS[M];
  return MWS[M];
}

double TicksToMeters(int Ticks){
  return (Ticks*3.14*WheelDiameter)/TPR;
}

//// Motor function code: 
void MotorWrite()
{
    // For the right motor
    if (WCS[0] > 0){
        digitalWrite(inApin[0], HIGH);
        digitalWrite(inApin[2], HIGH);
    }else{
        digitalWrite(inApin[0], LOW);
        digitalWrite(inApin[2], LOW);
    }
    double MSpeed0 = CorrectedSpeed(0, WCS[0]);
    Mspeeds[0]=MSpeed0;
    analogWrite(pwmpin[0], int(MSpeed0));
    analogWrite(pwmpin[2], int(MSpeed0));
        
    // For the left motor 
    if (WCS[1] > 0){
        digitalWrite(inApin[1], HIGH);
        digitalWrite(inApin[3], HIGH);
    }else{
        digitalWrite(inApin[1], LOW);
        digitalWrite(inApin[3], LOW);
    }
    double MSpeed1 = CorrectedSpeed(1, WCS[1]);
    Mspeeds[1]=MSpeed1;
    analogWrite(pwmpin[1], int(MSpeed1));
    analogWrite(pwmpin[3], int(MSpeed1));
}
