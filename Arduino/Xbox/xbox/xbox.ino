/*
  ============================================
  UFMG BIA-Kit is placed under the MIT License
  Copyright (c) 2024 by GTI (UFMG)
  ============================================
  Hardware:
  * ESP-WROOM-32 30 pin development board
  * 2 Nidec 24-H
  * 2 Set Tire wheel 88516 + 88517
  * 1 2GT-280 belt, width 6mm
  * 1 All-Metal Gear Servo SG90
  * 1 DC(12V)-DC(5V) Adjustable Linear Regulator 
  * 1 Module GY-521 MPU-6050
  * 1 Power Bank Box Charger, DC 12V output, 3x18650 batteries
*/

// I2C libray communication
#include <Wire.h>
 
// ENCODER library based on the built in counter hardware
#include <ESP32Encoder.h>

// ESP32 BLUE LED pin
#define INTERNAL_LED 2

// IMU I2C address
#define MPU   0x68

// SERVO pin
#define SERVO_PWM 32
#define STEERING_CENTER 76
#define ANGULO_MAX 32
float steering = 0;

// SERVO PWM config
#define SERVO_TIMER_BIT 16
#define SERVO_BASE_FREQ 50
#define SERVO_PWM_CH 3

// NIDEC PWM config
#define NIDEC_TIMER_BIT   8
#define NIDEC_BASE_FREQ   20000

// NIDEC pins: Reaction Wheel Motor
#define BRAKE1             4 //Yellow wire (Start/Stop)
#define NIDEC1_PWM        16 //Write wire  (PWM)
#define DIR1              17 //Green wire  (Forward/Reverse) 
#define ENCA_1            18 //Brown wire  (Signal A)  
#define ENCB_1            19 //Orange      (Signal B)  
#define NIDEC1_PWM_CH      1

// NIDEC pins: Traction Wheel Motor
#define BRAKE2           14 //Yellow wire (Start/Stop)
#define NIDEC2_PWM       27 //Write wire  (PWM)
#define DIR2             26 //Green wire  (Forward/Reverse) 
#define ENCA_2           25 //Brown wire // purple (Signal A)
#define ENCB_2           33 //Orange   (Signal B)
#define NIDEC2_PWM_CH     0

// Xbox controller conection
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;

// Encoder vars
ESP32Encoder NIDEC1_ENC;
ESP32Encoder NIDEC2_ENC;

// Kalman Filter vars
float Q_angle = 0.001; // Angular data confidence
float Q_bias  = 0.005; // Angular velocity data confidence
float R_meas  = 1.0;
float roll_angle = 0.0;
float bias = 0.0;
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float K[2] = {0, 0};

// Control vars
//     theta         dthehta         omega           integral 
float K1 = -282,    K2 = -24.1,    K3 = -0.135,    K4 = 0.0104;
float U = 0, U2 = 0;
int pwm = 0, pwm2 = 0;
float theta = 0.0, theta_dot = 0.0;            // System states
float integral = 0, omega = 0, integral_past = 0;
float bike_position = 0, bike_speed = 0.0;     
float Ts = 0.01, currentT = 0.0, previousT = 0.0;        // Elapsed time in loop() function
float Tb = 0.11, currentTb = 0.0, previousTb = 0.0;
float Txbox = 0.3, currentX = 0.0, previousX = 0.0;
float servoTime1=0, servoTime2=-60;

// Xbox vars
float analogR = 0, analogL = 0;
int marcha = 2, marchaDist = 160;
bool reset = false;
int impulseR = 0, impulseL = 0;
bool manobra = false, fullReto = false;

// Variáveis auxiliares
int etapa = 1;
float etapaAnteriorT, etapaAtualT;
float erro=0, erro_integral=0;
float kp=10, ki=25;
bool waiting = false;
float curvaVel=30, retoVel=45, vel_ref = 0;
int turning = 4;
float distLOW=60;
int alterna = 1;

// MAIN SETUP
void setup() { // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);                   // make sure your Serial Monitor is also set at this baud rate.

  NIDECsetup();
  IMUsetup();
  SERVOsetup();

  pinMode(INTERNAL_LED,OUTPUT);
  digitalWrite(INTERNAL_LED,HIGH);  // Turn on blue led
  delay(1500);                      // Wait for the system to stabilize
  for (int i=1; i<= 400; i++){      // Wait for the Kalman filter stabilize
    IMUread();
    delay(5);
  }
  currentT = millis();
  digitalWrite(INTERNAL_LED,LOW);

  xboxController.begin();  
}

// MAIN LOOP
void loop() {// put your main code here, to run repeatedly:
  equilibra();
  xbox();
  if(manobra) trick();
  else {
    etapa = 1;
    waiting = false;   
  }
}

void equilibra() {

  currentT = millis();
  if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;
    
    IMUread();

    if (abs(roll_angle) < 30 && !reset){   

      digitalWrite(BRAKE1, HIGH);
      digitalWrite(BRAKE2, HIGH);

      theta += theta_dot * Ts;
      integral = -NIDEC1_ENC.getCount()/63.7;
      omega = -(integral - integral_past)/Ts;
      integral_past = integral;

      U = -(K1*theta + K2*theta_dot + K3*omega + K4*integral);
      U = constrain(U, -12, 12);

      pwm = U*21.3 + impulseL - impulseR;
      MOTOR1cmd(pwm);
      
      bike_speed = -NIDEC2_ENC.getCount()/(Ts*6.12); // mm/s
      NIDEC2_ENC.clearCount();
      if(manobra){
        velControl();
      } else {
        MOTOR2cmd(-analogL);
        SERVOangle(STEERING_CENTER + steering);
      }

    } else {
      digitalWrite(BRAKE1, LOW); // stop reaction wheel
      digitalWrite(BRAKE2, LOW); // stop traction wheel
      SERVOangle(STEERING_CENTER);
      digitalWrite(INTERNAL_LED,HIGH);
      delay(5000);       // time to lift the bike
      digitalWrite(INTERNAL_LED,LOW);
      for (int i=1; i<= 400; i++){ //Wait for the Kalman Filter stabilize
        IMUread();
        delay(5);
      }
      previousT = millis();
      theta = 0.0;
      integral = 0.0;
      NIDEC1_ENC.clearCount();
      reset = false;
    }
  }   

}

void xbox() {
  currentX = millis();
  if ((currentX - previousX)/1000.0 >= Txbox) {
    previousX = currentX;
    
    xboxController.onLoop();

    if (xboxController.isWaitingForFirstNotification()) {
    } else {
      uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
      analogR = -((float)xboxController.xboxNotif.joyRHori - 32768)/2000;
      analogL = -((float)xboxController.xboxNotif.joyLVert - 32768)/(128 + (marchaDist * (5 - marcha)));

      marcha = xboxController.xboxNotif.btnRB && xboxController.xboxNotif.btnLB? 5 : marcha;
      marcha = xboxController.xboxNotif.btnX ? 3 : marcha;
      marcha = xboxController.xboxNotif.btnA ? 2 : marcha;
      marcha = xboxController.xboxNotif.btnB ? 1 : marcha;

      impulseR = xboxController.xboxNotif.trigRT/5;
      impulseL = xboxController.xboxNotif.trigLT/5;

      manobra = xboxController.xboxNotif.btnRS? true : manobra;
      manobra = xboxController.xboxNotif.btnLS? false : manobra;

      fullReto = xboxController.xboxNotif.btnDirUp? true : fullReto;
      fullReto = xboxController.xboxNotif.btnDirDown? false : fullReto;

      reset = xboxController.xboxNotif.btnStart? true : reset;

      if (fullReto) steering=0;
      else steering = constrain(steering + analogR, -ANGULO_MAX, ANGULO_MAX);  
    }
    
    if (pwm > 200) {
      xboxVibrationRight();
    } else if(pwm < -200) {
      xboxVibrationLeft();
    }

  }   
}

void xboxVibrationRight() {
  repo.v.select.center = false;
  repo.v.select.left = false;
  repo.v.select.right = true;
  repo.v.select.shake = false;
  repo.v.power.right = 50;
  repo.v.timeActive = Txbox*100;
  xboxController.writeHIDReport(repo);
}

void xboxVibrationLeft() {
  repo.v.select.center = false;
  repo.v.select.left = true;
  repo.v.select.right = false;
  repo.v.select.shake = false;
  repo.v.power.left = 50;
  repo.v.timeActive = Txbox*100;
  xboxController.writeHIDReport(repo);
}

void trick(){
  currentTb = millis();
  if ((currentTb - previousTb)/1000.0 >= Tb) {
    previousTb = currentTb;

    if (!waiting) {
      switch (etapa) {
        case 1: // Espera um tempo antes de começar a andar
          wait();
          etapa = 2;
          break;

        case 2: // Vira o guidão
          steering += turning;
          steering = constrain(steering, -ANGULO_MAX, ANGULO_MAX);
          SERVOangle(steering + STEERING_CENTER);
          if(steering >= ANGULO_MAX || steering <= -ANGULO_MAX){
            wait();
            etapa = 3;
          }
          break;
        
        case 3:
          vel_ref = -0.4;
          if ((currentTb - etapaAnteriorT)/1000>=3.5) {
            wait();
            etapa = 4;
          }          
          break;

        case 4: // Vira o guidão
          steering -= turning;
          steering = constrain(steering, -ANGULO_MAX, ANGULO_MAX);
          SERVOangle(steering + STEERING_CENTER);
          if(steering >= ANGULO_MAX || steering <= -ANGULO_MAX){
            wait();
            etapa = 5;
          }
          break;

        case 5:
          vel_ref = 0.4;
          if ((currentTb - etapaAnteriorT)/1000>=3.5) {
            wait();
            etapa = 2;
          }          
          break;

      }
    } else {
      vel_ref = 0;
      if ((currentTb - etapaAnteriorT)/1000>=1.5) waiting = false;
    }

  }
}

// SETUP functions
void IMUsetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                      //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                      //We want to write to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00000000);             //Set the register bits as 00000000 (250dps full scale), 00010000 (1000dps full scale)
  Wire.write(1 << 3);
  Wire.endTransmission();                //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU);           //Start communication with the address found during search.
  Wire.write(0x1C);                      //We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                //Set the register bits as 00000000 (+/- 2g full scale range), 00010000 (+/- 8g full scale range)
  Wire.endTransmission(); 
}

void SERVOsetup(){
  pinMode(SERVO_PWM, OUTPUT);
  ledcSetup(SERVO_PWM_CH, SERVO_BASE_FREQ, SERVO_TIMER_BIT);
  ledcAttachPin(SERVO_PWM, SERVO_PWM_CH);
  SERVOangle(STEERING_CENTER);                   // SERVO initial position
}

void NIDECsetup(){
  pinMode(BRAKE1, OUTPUT);
  digitalWrite(BRAKE1, HIGH);

  pinMode(BRAKE2, OUTPUT);
  digitalWrite(BRAKE2, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(NIDEC1_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC1_PWM, NIDEC1_PWM_CH);
  MOTOR1cmd(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(NIDEC2_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC2_PWM, NIDEC2_PWM_CH);
  MOTOR2cmd(0);

  // Encoders setup:
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Encoder 1:
	NIDEC1_ENC.attachFullQuad(ENCB_1, ENCA_1);
  NIDEC1_ENC.clearCount();
  // Encoder 2:
	NIDEC2_ENC.attachFullQuad(ENCB_2, ENCA_2);
  NIDEC2_ENC.clearCount();
}

// IMU function: Kalman Filter
void IMUread(){
  // read IMU
  int16_t ax,ay,az,temp,gx,gy,gz;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);        // IMU address: 0x68
  ax=Wire.read()<<8|Wire.read();   // X-axis value: 16384.0; 
  ay=Wire.read()<<8|Wire.read();   // Y-axis value: 16384.0;     
  az=Wire.read()<<8|Wire.read();   // Z-axis value: 16384.0;  
  temp=Wire.read()<<8|Wire.read();      
  gx=Wire.read()<<8|Wire.read();  
  gy=Wire.read()<<8|Wire.read();  
  gz=Wire.read()<<8|Wire.read();  
  //accelerometer angles in degrees (or rads)
  float ax_angle = atan2(ay, sqrt(ax*ax + az*az)) * 57.3; // roll
  //float ay_angle = atan2(ax, sqrt(ay*ay + az*az)) * 57.3; // pitch
  // float az_angle = atan2(sqrt(ax*ax + az*az), az) * 57.3; // yaw (useless)
  // gyro measurements in degress (or rads)
  gx =  gx / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gy =  gy / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gz =  gz / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  
  // begin: Kalman filter - Roll Axis (X)
  roll_angle += (gx - bias) * Ts;
  
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;
  //
  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);  
  //
  roll_angle += K[0] * (ax_angle - roll_angle); 
  bias       += K[1] * (ax_angle - roll_angle);
  //
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  // end: Kalman filter 

  theta_dot = (gx - bias)/57.3; // Unbiased gyro speed
  
  // //  Complementary filter   
  // roll_anglec = 0.98 * (roll_anglec + gx * Ts) + 0.02 * ax_angle;
  // pitch_anglec = 0.98 * (pitch_anglec + gy * Ts) + 0.02 * ay_angle;
  // yaw_anglec = 0.98 * (yaw_anglec + gz * Ts) + 0.02 * az_angle;

}  

// NIDEC functions
void MOTOR1cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR1, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR1, LOW);
  }
  ledcWrite(NIDEC1_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

void MOTOR2cmd(int sp) {
  sp = - sp;
  if (sp < 0) {
    digitalWrite(DIR2, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR2, LOW);
  }
  ledcWrite(NIDEC2_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

//SERVO function
void SERVOangle(float angle) {
  float dutyCycle = angle*(6400/180) + 1500;
  ledcWrite(SERVO_PWM_CH, int(dutyCycle));  
}

void velControl() {
  erro = vel_ref - (bike_speed/1000); // bike speed in mm/s to m/s
  erro_integral += erro*Ts;
  U2 = kp*erro + ki*erro_integral;
  pwm2 = U2*21.3;
  MOTOR2cmd(pwm2);
}

void wait() {
  etapaAnteriorT = millis();
  waiting = true;
  erro_integral = 0;
}
