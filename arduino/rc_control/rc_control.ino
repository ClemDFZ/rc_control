
/**
 * @file rc_control.ino
 * @author clemDFZ
 * @version 0.1
 * @date 2024-05-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <CrsfSerial.h>
#include "Motor.h"
#include "Car.h"
// Pin interrupters for motor encoder
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#if defined(__AVR_ATmega2560__)
CrsfSerial crsf(Serial2, CRSF_BAUDRATE);
#else
#error NOT MEGA2560
#endif


#include "Simple_MPU6050.h"
#include "MPU_handler.cpp"
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

Simple_MPU6050 mpu;
MPU_handler mpu_handler;

float get_yaw(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float eulerDEG[3];         // [psi, theta, phi]    Euler angle container
  //spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
  mpu.GetQuaternion(&q, quat);
  mpu.GetEuler(euler, &q);
  mpu.ConvertToDegrees(euler, eulerDEG);
  //Serial.println(eulerDEG[0]);
  return eulerDEG[0];
  //}
}

void update_handler (int16_t *gyro, int16_t *accel, int32_t *quat) {
  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM
  float yaw = get_yaw(quat, Spam_Delay);
  mpu_handler.update_measure(yaw);
}




// Distances inter-moteurs
float L = 0.25; // wheels length distance
float W = 0.2;  // wheels width distance
float wheel_radius=(97.0/2.0)/1000.0; //wheel radius (meter)

// Gestion fréquence boucle
const unsigned long SAMPLING_FREQUENCY = 100; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long SAMPLING_PERIOD = 1000 / SAMPLING_FREQUENCY; // Période en millisecondes
unsigned long lastSAMPLINGTime = 0;


const unsigned long PID_FREQUENCY = 10; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long PID_PERIOD = 1000 / PID_FREQUENCY; // Période en millisecondes
unsigned long lastPIDTime = 0;


const unsigned long DISPLAY_FREQUENCY = 25; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long DISPLAY_PERIOD = 1000 / DISPLAY_FREQUENCY; // Période en millisecondes
unsigned long lastDISPLAYTime = 0;

const int SENSOR_AVG_FACTOR = SAMPLING_FREQUENCY/PID_FREQUENCY;

const unsigned long SETPOINT_FREQUENCY = 1; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long SETPOINT_PERIOD = 1000 / SETPOINT_FREQUENCY; // Période en millisecondes
unsigned long lastSetpointTime = 0;


const float HIGH_SETPOINT_ROUND_PER_SEC = -1.0;
const float LOW_SETPOINT_ROUND_PER_SEC = 2.0;

const float HIGH_SETPOINT_RAD_PER_SEC = HIGH_SETPOINT_ROUND_PER_SEC*2*PI;
const float LOW_SETPOINT_RAD_PER_SEC = LOW_SETPOINT_ROUND_PER_SEC*2*PI;

const float LOW_SETPOINT_METER_SEC = 0.3;
const float HIGH_SETPOINT_METER_SEC = 0.5;

int PWM_setpoint = 25;

float lastSetpoint = LOW_SETPOINT_METER_SEC;


Car Mecanum_Car(SAMPLING_PERIOD);

// Gestion RC
void RC_callback();
int throttle,roll,pitch,yaw,LB,RB,LP;
bool RT = true;
bool LT = true;

String control_mode = "SERIAL";
String receivedData="";

float Vx_setpoint = 0.0;
float Vy_setpoint = 0.0;
float omegaZ_setpoint = 0.0;



bool Serial_Connected = false ;

void setup() {
  /*
  TCCR3B = TCCR3B & 0b11111000 | 0x02;
  TCCR4B = TCCR4B & 0b11111000 | 0x02;  // Prescaler 8 -> PWM ~3.9 kHz
  */
  Serial.begin(115200);
  delay(10);
  if (Serial)
  {
    Serial_Connected = true;
  }
  /*
  while (!Serial) {
      ; // Attendre que la connexion série soit établie (utile lors de l'utilisation de certaines cartes comme l'UNO)
    }
  */
  crsf.begin();
  crsf.onPacketChannels = &RC_callback;
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);           // Set the DMP output rate from 200Hz to 5 Minutes.
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.CalibrateMPU();
  mpu.load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(update_handler);
  // Initialiser la pin d'interruption pour Encoder A
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(62), tachy_front_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(64), tachy_front_right, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(66), tachy_rear_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(68), tachy_rear_right, CHANGE);
  control_mode = "SERIAL";
}

void tachy_front_left(){
  Mecanum_Car.tachy_front_left();
}
void tachy_front_right(){
  Mecanum_Car.tachy_front_right();
}
void tachy_rear_left(){
  Mecanum_Car.tachy_rear_left();
}
void tachy_rear_right(){
  Mecanum_Car.tachy_rear_right();
}

//////////////////////////////////////////////////////////////////////////////////
void loop() { 
  unsigned long t0 = millis();
  crsf.loop(); 
  mpu.dmp_read_fifo();// Must be in loop  Interrupt pin must be connected
  if (Serial_Connected){
    readSerial();
  }
  if  (control_mode=="SERIAL") {
    //Mecanum_Car.send_PWM(PWM_setpoint,0,0,0);
    unsigned long now_sampling = micros();
    if (now_sampling - lastSAMPLINGTime >= SAMPLING_PERIOD*1000.0) {  
      lastSAMPLINGTime = now_sampling;
      update_sensors();
  
      
      
      
      unsigned long now_PID = millis();
      if (now_PID - lastPIDTime >= PID_PERIOD) {
          lastPIDTime = now_PID; // Mettre à jour le dernier temps PID
          // Calcul du PID
          float avg_yaw = mpu_handler.get_averaged_speed();
          Mecanum_Car.forward_kinematics(avg_yaw);       
          Mecanum_Car.update_velocity_PID();
          Mecanum_Car.inverse_kinematics();
          
          
      Mecanum_Car.update_motor_PID();
      Mecanum_Car.update_motors_command();    
      //Mecanum_Car.display_motor_speed();
/*
      Serial.print(PWM_setpoint);
      Serial.print(",");
      Mecanum_Car.display_motor_speed(); 
  */    
      unsigned long now_display = millis();
      if (now_display - lastDISPLAYTime >= DISPLAY_PERIOD) {
        lastDISPLAYTime = now_display;
        /*
        Serial.print(0);
        Serial.print(",");
        Serial.print(20);
        Serial.print(",");  
        */     
        //Mecanum_Car.display_motor_speed(); 
        //Mecanum_Car.display_fwd_kinematics();
      }

      unsigned long now_setpoint = millis();
      if (now_setpoint - lastSetpointTime >= SETPOINT_PERIOD) {
        update_setpoint(now_setpoint);
      }
      /*
      unsigned long deltaT = millis()-t0;
      if (deltaT < SAMPLING_PERIOD) {
        delay(SAMPLING_PERIOD - deltaT);
      
      */
      }
  }
  }
}

void update_setpoint(unsigned long now_setpoint){
    if (lastSetpoint>=HIGH_SETPOINT_METER_SEC)
    {      
      lastSetpoint = LOW_SETPOINT_METER_SEC;
    }
    else
    {
      //lastSetpoint += 0.1;
      lastSetpoint = HIGH_SETPOINT_METER_SEC;
    }
    PWM_setpoint++;
    
    lastSetpointTime = now_setpoint;
    Vx_setpoint = lastSetpoint;
    Mecanum_Car.set_car_setpoints(Vx_setpoint,0.0,0.0);
}

void update_sensors(){
  Mecanum_Car.update_tachy();
}

void readSerial() // Lecture des données envoyées sur le port Série 0
{
  if (Serial.available() > 0) {
      char receivedChar = Serial.read();  // Lire un caractère
      if (receivedChar == '\n') {  // Fin de la ligne (si utilisé comme délimiteur)
        parseString(receivedData);
        receivedData = "";  // Réinitialiser pour la prochaine chaîne
      } else {
        receivedData += receivedChar;  // Ajouter le caractère à la chaîne reçue
      }
    }
}

void parseString(String str) {
  // Chercher les positions des délimiteurs (espaces)
  int xIndex = str.indexOf('X');
  int yIndex = str.indexOf('Y');
  int zIndex = str.indexOf('Z');

  int pIndex = str.indexOf('P');
  int iIndex = str.indexOf('I'); 
  int dIndex = str.indexOf('D');

  // Extraire les sous-chaînes pour X, Y, Z
  if (xIndex != -1) {
    int spaceIndex = str.indexOf(' ', xIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'X' est le dernier élément
    Vx_setpoint = str.substring(xIndex + 1, spaceIndex).toFloat();
  }

  if (yIndex != -1) {
    int spaceIndex = str.indexOf(' ', yIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Y' est le dernier élément
    Vy_setpoint = str.substring(yIndex + 1, spaceIndex).toFloat();
  }

  if (zIndex != -1) {
    int spaceIndex = str.indexOf(' ', zIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    omegaZ_setpoint = str.substring(zIndex + 1, spaceIndex).toFloat();
  }

  if (pIndex != -1) {
    int spaceIndex = str.indexOf(' ', pIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Kp = str.substring(pIndex + 1, spaceIndex).toFloat();
    //Mecanum_Car.set_Kp(Kp);
    Mecanum_Car.set_motor_Kp(Kp);
  }

  if (iIndex != -1) {
    int spaceIndex = str.indexOf(' ', iIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Ki = str.substring(iIndex + 1, spaceIndex).toFloat();
    //Mecanum_Car.set_Ki(Ki);
    Mecanum_Car.set_motor_Ki(Ki);
  }

  if (dIndex != -1) {
    int spaceIndex = str.indexOf(' ', dIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Kd = str.substring(dIndex + 1, spaceIndex).toFloat();
    Mecanum_Car.set_motor_Kd(Kd);

  }


  }

void RC_callback() {
  
  throttle = crsf.getChannel(1);
  throttle = constrain(map(throttle,1000,2000,-100,100), -100, 100);
  roll = crsf.getChannel(2);
  roll = -constrain(map(roll,1000,2000,-100,100), -100, 100);
  pitch= crsf.getChannel(3);
  pitch = constrain(map(pitch,1000,2000,-100,100), -100, 100);
  yaw= crsf.getChannel(4);
  yaw = -constrain(map(yaw,1000,2000,-100,100), -100, 100);  
  LT= single_switch(crsf.getChannel(5));
  if (RT){
    control_mode="RC";
  }else{
    control_mode="SERIAL";
  }
  RT= single_switch(crsf.getChannel(6));
  LB= double_switch(crsf.getChannel(7));
  RB= double_switch(crsf.getChannel(8));
  LP= crsf.getChannel(9);  

/*
  Serial.print(throttle);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");  
  Serial.print(pitch);
  Serial.print(",");  
  Serial.print(yaw);
  Serial.print(",");  
  Serial.print(LT);  
  Serial.print(",");  
  Serial.print(RT);  
  Serial.print(",");  
  Serial.print(LB);
  Serial.print(",");  
  Serial.print(RB);
  Serial.print(",");  
  Serial.print(LP);
  Serial.println();
*/
  control_mode="SERIAL";

  if (control_mode == "RC")
    {
    float omega1 = (pitch - roll - (L + W) * yaw);
    float omega2 = (pitch + roll + (L + W) * yaw);
    float omega3 = (pitch + roll - (L + W) * yaw);
    float omega4 = (pitch - roll + (L + W) * yaw);
    float maxSpeed = max(max(abs(omega1), abs(omega2)), max(abs(omega3), abs(omega4)));
    float maxThrottle = max((float)max(abs(roll),(float)abs(pitch)),(float)abs(yaw))/100;
    // Normaliser les vitesses des roues pour qu'elles se situent entre -255 et 255
    if (maxSpeed > 0) 
      {
      float PWM1 = omega1 / maxSpeed * 255 * maxThrottle  ;
      float PWM2 = omega2 / maxSpeed * 255 * maxThrottle  ;
      float PWM3 = omega3 / maxSpeed * 255 * maxThrottle  ;
      float PWM4 = omega4 / maxSpeed * 255 * maxThrottle  ;
      //Mecanum_Car.send_PWM(PWM1,PWM2,PWM3,PWM4);
      }
    } 
}

bool single_switch(int value) {
    if (value < 1500) {
        return 0;
    }
    else{
        return 1;
    }
}

int double_switch(int value) {
    if (value <= 1300) {
        return 0;
    } else if ((1300<value) && (value<1800)) {
        return 1;
    } else if (value >= 1800) {
        return 2;
    } else {
        // Valeur par défaut si la valeur n'est pas parmi celles attendues
        return -1;  // Ou une autre valeur d'erreur appropriée
    }
}


void mpu_setup(Simple_MPU6050 mpu){
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(100);           // Set the DMP output rate from 200Hz to 5 Minutes.
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.CalibrateMPU();
  mpu.load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(update_handler);
}