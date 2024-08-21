
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
// Pin interrupters for motor encoder
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#if defined(__AVR_ATmega2560__)
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);
#else
#error NOT MEGA2560
#endif


// Distances inter-moteurs
float L = 0.25; // wheels length distance
float W = 0.2;  // wheels width distance
float wheel_radius=(97.0/2.0)/1000.0; //wheel radius (meter)

// Gestion fréquence boucle
const unsigned long SAMPLING_FREQUENCY = 100; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long SAMPLING_PERIOD = 1000 / SAMPLING_FREQUENCY; // Période en millisecondes

const unsigned long PID_FREQUENCY = 100; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long PID_PERIOD = 1000 / PID_FREQUENCY; // Période en millisecondes
unsigned long lastPIDTime = 0;

float Kp_Vx = 0.3;
float Ki_Vx = 0.01;
float Kp_Vy = 0.3;
float Ki_Vy = 0.01;
float Kp_omegaZ = 0.1;
float Ki_omegaZ = 0.001;

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

const float LOW_SETPOINT_METER_SEC = 0.2;
const float HIGH_SETPOINT_METER_SEC = 0.5;

float lastSetpoint = LOW_SETPOINT_METER_SEC;


// Création moteurs et attribution PIN
Motor FrontLeft_motor(52, 53, 4, 62, 63,SAMPLING_PERIOD);    // A8 -> 62, A9 -> 63
Motor FrontRight_motor(50, 51, 5, 64, 65,SAMPLING_PERIOD);   // A10 -> 64, A11 -> 65
Motor RearLeft_motor(26, 27, 6, 66, 67,SAMPLING_PERIOD);     // A12 -> 66, A13 -> 67
Motor RearRight_motor(24, 25, 7, 68, 69,SAMPLING_PERIOD);    // A14 -> 68, A15 -> 69
Motor* motors_list[4] = { &FrontLeft_motor, &FrontRight_motor, &RearLeft_motor,&RearRight_motor};
//Motor* motors_list[1] = {&RearRight_motor};
size_t motors_list_length = sizeof(motors_list) / sizeof(motors_list[0]);

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

float Vx_corrected,Vy_corrected,omegaZ_corrected;
float Vx_odo,Vy_odo,omegaZ_odo;

bool Serial_Connected = false ;

/**
 * @brief Executed once on power-up or reboot
 */
void setup() {
  
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

  // Initialiser la pin d'interruption pour Encoder A
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(62), tachy_front_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(64), tachy_front_right, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(66), tachy_rear_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(68), tachy_rear_right, CHANGE);

}

//////////////////////////////////////////////////////////////////////////////////
void loop() { 
  unsigned long t0 = millis();
  crsf.loop(); 
  if (Serial_Connected){
    readSerial();
  }
  if(control_mode == "RC") {
    RC_commands();
  } 
  else if (control_mode=="SERIAL")
  {
    inverse_kinematics();
  }
  update_sensors();
  forward_kinematics();
  
  unsigned long now_PID = millis();
  if (now_PID - lastPIDTime >= PID_PERIOD) {
      lastPIDTime = now_PID; // Mettre à jour le dernier temps PID
      // Calcul du PID
      update_velocity_PID();
      
  }
  update_motor_PID();



  unsigned long now_display = millis();
  if (now_display - lastDISPLAYTime >= DISPLAY_PERIOD) {
    lastDISPLAYTime = now_display;
    //display_motor_speed();
    display_fwd_kinematics();
  }

  unsigned long now_setpoint = millis();
  if (now_setpoint - lastSetpointTime >= SETPOINT_PERIOD) {
    update_setpoint(now_setpoint);
  }

//////////////////////////////////////////////////////////////////////////////////
  update_motors_command();
  unsigned long deltaT = millis()-t0;
  if (deltaT < SAMPLING_PERIOD) {
    delay(SAMPLING_PERIOD - deltaT);
  }
  }

void update_setpoint(unsigned long now_setpoint){
    if (lastSetpoint==LOW_SETPOINT_METER_SEC)
    {
      lastSetpoint = HIGH_SETPOINT_METER_SEC;
    }
    else
    {
      lastSetpoint = LOW_SETPOINT_METER_SEC;
    }
    lastSetpointTime = now_setpoint;
    Vx_setpoint = lastSetpoint;
}

void update_motors_command()
{
  for (int i=0;i<motors_list_length;i++)
  {
    motors_list[i]->send_PID_input();
    //motors_list[i]->sendPWM(-255);
  }
  /*
  for (int i=0;i<motors_list_length;i++)
  {
    motors_list[i]->sendPWM(255);
  }
  */

}

void update_motor_PID()
{
  for (int i=0;i<motors_list_length;i++)
  {
    motors_list[i]->PID_controller();
  }
  //RearRight_motor.print_commands();
}

void update_sensors()
{
  for (int i=0;i<motors_list_length;i++)
  {
    motors_list[i]->update_rotation_speed();
  }
}

void tachy_front_left()
{
  FrontLeft_motor.increase_tachy();
}

void tachy_front_right()
{
  FrontRight_motor.increase_tachy();
}

void tachy_rear_left()
{
  RearLeft_motor.increase_tachy();
}

void tachy_rear_right()
{
  RearRight_motor.increase_tachy();
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
    Kp_Vx = Kp;
    /*
    for (int i=0;i<motors_list_length;i++)
    {
      motors_list[i]->set_Kp(Kp);
    }
    */
  }

  if (iIndex != -1) {
    int spaceIndex = str.indexOf(' ', iIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Ki = str.substring(iIndex + 1, spaceIndex).toFloat();
    Ki_Vx = Ki;
    /*
    for (int i=0;i<motors_list_length;i++)
      {
        motors_list[i]->set_Ki(Ki);
      }
      */
  }

  if (dIndex != -1) {
    int spaceIndex = str.indexOf(' ', dIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Kd = str.substring(dIndex + 1, spaceIndex).toFloat();
    for (int i=0;i<motors_list_length;i++)
    {
      motors_list[i]->set_Kd(Kd);
    }
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
}

void update_commands(float omega1,float omega2,float omega3,float omega4){ 
  float omega_list[4] {omega1,omega2,omega3,omega4};
  for (int i=0;i<motors_list_length;i++)
  {
    motors_list[i]->setSpeed(omega_list[i]);
  }



 /* int pwmFL = constrain(omega1,-255,255); // Convertir float en int
  int pwmFR = constrain(omega2,-255,255); // Convertir float en int
  int pwmRL = constrain(omega3,-255,255); // Convertir float en int
  int pwmRR = constrain(omega4,-255,255); // Convertir float en int

  if (!LT){
    FrontLeft_motor.sendPWM(0);
    FrontRight_motor.sendPWM(0);
    RearLeft_motor.sendPWM(0);
    RearRight_motor.sendPWM(0);
  }
  else {
    FrontLeft_motor.sendPWM(pwmFL);
    FrontRight_motor.sendPWM(pwmFR);
    RearLeft_motor.sendPWM(pwmRL);
    RearRight_motor.sendPWM(pwmRR);
  }

 */
  
}

void RC_commands(){
  Vx_setpoint = pitch;
  Vy_setpoint = roll ; 
  omegaZ_setpoint = yaw; 
  // Calcul des vitesses des roues
  float omega1 = (Vx_setpoint - Vy_setpoint - (L + W) * omegaZ_setpoint);
  float omega2 = (Vx_setpoint + Vy_setpoint + (L + W) * omegaZ_setpoint);
  float omega3 = (Vx_setpoint + Vy_setpoint - (L + W) * omegaZ_setpoint);
  float omega4 = (Vx_setpoint - Vy_setpoint + (L + W) * omegaZ_setpoint);
  float maxSpeed = max(max(abs(omega1), abs(omega2)), max(abs(omega3), abs(omega4)));
  float maxThrottle = max((float)max(abs(Vy_setpoint),(float)abs(Vx_setpoint)),(float)abs(omegaZ_setpoint))/100;
  // Normaliser les vitesses des roues pour qu'elles se situent entre -255 et 255
  if (maxSpeed > 0) {
    omega1 = omega1 / maxSpeed * 255 * maxThrottle  ;
    omega2 = omega2 / maxSpeed * 255 * maxThrottle  ;
    omega3 = omega3 / maxSpeed * 255 * maxThrottle  ;
    omega4 = omega4 / maxSpeed * 255 * maxThrottle  ;
  }
  else{
    omega1 = 0;
    omega2 = 0;
    omega3 = 0;
    omega4 = 0;
  }
  update_commands(omega1,omega2,omega3,omega4);
}

void inverse_kinematics(){
  // Calculer les vitesses des roues
  float omega1 = (Vx_corrected - Vy_corrected - (L + W) * omegaZ_corrected)/wheel_radius;
  float omega2 = (Vx_corrected + Vy_corrected + (L + W) * omegaZ_corrected)/wheel_radius;
  float omega3 = (Vx_corrected + Vy_corrected - (L + W) * omegaZ_corrected)/wheel_radius;
  float omega4 = (Vx_corrected - Vy_corrected + (L + W) * omegaZ_corrected)/wheel_radius;

  /*
  Serial.println();
  Serial.println(wheel_radius);
  Serial.println(Vx_corrected);
  Serial.println(Vy_corrected);
  Serial.println(omegaZ_corrected);
  Serial.println(omega1);
  */

  update_commands(omega1,omega2,omega3,omega4);
}

void forward_kinematics(){
  // Calculer les vitesses des roues
  float omega1 = FrontLeft_motor.getRotationSpeed();
  float omega2 = FrontRight_motor.getRotationSpeed();
  float omega3 = RearLeft_motor.getRotationSpeed();
  float omega4 = RearRight_motor.getRotationSpeed();
  Vx_odo = wheel_radius*(omega1+omega2+omega3+omega4)/4;
  Vy_odo = wheel_radius*(-omega1+omega2+omega3-omega4)/4;
  omegaZ_odo = wheel_radius*(-omega1+omega2-omega3+omega4)/(4*(L+W));
}


void update_velocity_PID()
{       
    //Serial.println("=============");
    float d_Vx = Vx_setpoint-Vx_odo;
    Vx_corrected = Vx_setpoint+ Kp_Vx*d_Vx;

    float d_Vy = Vy_odo-Vy_setpoint;
    Vy_corrected = Vy_setpoint+ Kp_Vy*d_Vy;

    float d_omegaZ = omegaZ_odo-omegaZ_setpoint;
    omegaZ_corrected = omegaZ_setpoint+ Kp_omegaZ*d_omegaZ;
}

void display_fwd_kinematics(){
  /**/
  Serial.print(Vx_setpoint);
  Serial.print(",");
  Serial.print(Vx_corrected);
  Serial.print(",");
  Serial.print(Vx_odo);
  Serial.println();
}

void display_motor_speed(){
  /**/
  FrontLeft_motor.print_commands();
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