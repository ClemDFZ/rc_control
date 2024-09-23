
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
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


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

#include <Servo.h> // Inclusion de la bibliothèque Servo

Servo cam_servo;
int servoPin = 10; // Pin où est connecté le servo
int angleServo = 45; 
const float SERVO_MILLI_PER_DEG = 150/60.0;
const int SAFE_SERVO_STARTUP_TIME=100;
bool servo_attached = false;
unsigned long timer_servo_not_idle;
unsigned long timer_servo_free = 0;
const unsigned int SERVO_IDLE_THRESHOLD = 2000; //Temps avant que le servo se detach
bool servo_used = false;
int servo_setpoint=45;
int target_angle = 45;

LiquidCrystal_I2C lcd_screen(0x27, 16, 2);

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


const unsigned long LCD_FREQUENCY = 2; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long LCD_PERIOD = 1000 / LCD_FREQUENCY; // Période en millisecondes
unsigned long lastLCDTime = 0;


const int SENSOR_AVG_FACTOR = SAMPLING_FREQUENCY/PID_FREQUENCY;

const unsigned long SETPOINT_FREQUENCY = 1; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long SETPOINT_PERIOD = 1000 / SETPOINT_FREQUENCY; // Période en millisecondes
unsigned long lastSetpointTime = 0;


const float HIGH_SETPOINT_ROUND_PER_SEC = -1.0;
const float LOW_SETPOINT_ROUND_PER_SEC = 2.0;

const float HIGH_SETPOINT_RAD_PER_SEC = HIGH_SETPOINT_ROUND_PER_SEC*2*PI;
const float LOW_SETPOINT_RAD_PER_SEC = LOW_SETPOINT_ROUND_PER_SEC*2*PI;

const float LOW_SETPOINT_METER_SEC = 0.5;
const float HIGH_SETPOINT_METER_SEC = 0.5;

int PWM_setpoint = 0;

float lastSetpoint = LOW_SETPOINT_METER_SEC;


Car Mecanum_Car(&mpu_handler,&lcd_screen,SAMPLING_PERIOD);

// Gestion RC
void RC_callback();
int throttle,roll,pitch,yaw,LB,RB,LP;
bool RT = true;
bool LT = true;

bool remote_controlled = false;
String receivedData="";

float Vx_setpoint = 0.0;
float Vy_setpoint = 0.0;
float omegaZ_setpoint = 0.0;

bool is_Armed = true;

bool Serial_Connected = false ;

void setup() {
  lcd_screen.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd_screen.backlight();
  lcd_screen.setCursor(2,0);
  lcd_screen.print("Calibrating");
  lcd_screen.setCursor(5,1);
  lcd_screen.print("IMU");
  cam_servo.attach(servoPin);
  cam_servo.write(angleServo);
  cam_servo.detach();
  timer_servo_not_idle = millis();
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
  lcd_screen.clear();
  lcd_screen.setCursor(0, 0); // Définir le curseur à la position (0,0)
  lcd_screen.print("Running"); 
  // Initialiser la pin d'interruption pour Encoder A
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(62), tachy_front_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(64), tachy_front_right, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(66), tachy_rear_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(68), tachy_rear_right, CHANGE);
  digitalWrite(52,HIGH); // LED to know calib is done
  Serial.println("setup end");
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
  unsigned long now_sampling = micros();
  if (now_sampling - lastSAMPLINGTime >= SAMPLING_PERIOD*1000.0) {  
      lastSAMPLINGTime = now_sampling;
      update_sensors(); 
  //Mecanum_Car.display_fwd_kinematics();

  unsigned long now_servo = millis();
  if (now_servo-timer_servo_not_idle >= timer_servo_free)
  {
    angleServo = target_angle;
    servo_used = false;
  }

  if (now_servo-timer_servo_not_idle >= SERVO_IDLE_THRESHOLD)
  {
      servo_attached = false;
      cam_servo.detach();
  }
  servo_handler();



  if  (!remote_controlled) {
    //Mecanum_Car.send_PWM(PWM_setpoint,0,0,0);
      
      unsigned long now_PID = millis();
      if (now_PID - lastPIDTime >= PID_PERIOD) {
          lastPIDTime = now_PID; // Mettre à jour le dernier temps PID
          // Calcul du PID
          Mecanum_Car.forward_kinematics();       
          Mecanum_Car.update_velocity_PID();
          Mecanum_Car.inverse_kinematics();
          
          
      Mecanum_Car.update_motor_PID();
      if (is_Armed){
       Mecanum_Car.update_motors_command();    
      }
      else{
        Mecanum_Car.send_PWM(0,0,0,0);
      }
      //Mecanum_Car.display_motor_speed();

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
    
      unsigned long now_lcd = millis();
      if (now_lcd - lastLCDTime >= LCD_PERIOD) {
        lastLCDTime = now_lcd;
        refresh_LCD();
      }



      unsigned long now_setpoint = millis();
      if (now_setpoint - lastSetpointTime >= SETPOINT_PERIOD) {
        lastSetpointTime = now_setpoint;
        //update_setpoint();
        /*
      Serial.print(PWM_setpoint);
      Serial.print(",");
      
      Mecanum_Car.display_motor_speed(); 
      */
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

void update_setpoint(){
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
    Vx_setpoint = lastSetpoint;
    Mecanum_Car.set_car_setpoints(Vx_setpoint,0.0,0.0);
    //Mecanum_Car.set_car_setpoints(0.0,0.0,PI/2);

}

void update_sensors(){
  Mecanum_Car.update_tachy();
}

void servo_handler(){
  if (!servo_used)
  {
    target_angle = servo_setpoint;
    if (target_angle != angleServo)
    {
      int diffAngle = abs(angleServo-target_angle);
      timer_servo_free = SERVO_MILLI_PER_DEG*diffAngle; 
      if (!servo_attached)
      {
        cam_servo.attach(servoPin);
        timer_servo_free += SAFE_SERVO_STARTUP_TIME; 
        servo_attached = true;
      } 
      cam_servo.write(target_angle);
      timer_servo_not_idle = millis();
      servo_used = true;
    }
  }
}

void readSerial() // Lecture des données envoyées sur le port Série 0
{
  if (Serial.available() > 0) {
      char receivedChar = Serial.read();  // Lire un caractère
      if (receivedChar == '\n') {  // Fin de la ligne (si utilisé comme délimiteur)
        parseString(receivedData);
        Serial.println("done");
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
  int sIndex = str.indexOf('S');

  bool update_setpoint = false;

  // Extraire les sous-chaînes pour X, Y, Z
  if (xIndex != -1) {
    int spaceIndex = str.indexOf(' ', xIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'X' est le dernier élément
    Vx_setpoint = str.substring(xIndex + 1, spaceIndex).toFloat();
    update_setpoint = true;
  }

  if (yIndex != -1) {
    int spaceIndex = str.indexOf(' ', yIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Y' est le dernier élément
    Vy_setpoint = str.substring(yIndex + 1, spaceIndex).toFloat();
    update_setpoint = true;
  }

  if (zIndex != -1) {
    int spaceIndex = str.indexOf(' ', zIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    omegaZ_setpoint = str.substring(zIndex + 1, spaceIndex).toFloat();
    update_setpoint = true;
  }

  if (sIndex != -1) {
    int spaceIndex = str.indexOf(' ', sIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    servo_setpoint = str.substring(sIndex + 1, spaceIndex).toInt();
  }

  if (update_setpoint){Mecanum_Car.set_car_setpoints(Vx_setpoint,Vy_setpoint,omegaZ_setpoint);}
  }

void refresh_LCD(){
  lcd_screen.clear();
  
  // Print setpoints Vx and Vy (m/s)
  String Vx_lcd = "X:"+String(Mecanum_Car.get_Vx()).substring(0, 6);
  String Vy_lcd = "Y:"+String(Mecanum_Car.get_Vy()).substring(0, 6);
  lcd_screen.setCursor(0, 0); // Définir le curseur à la position (0,0)
  lcd_screen.print(Vx_lcd); 
  lcd_screen.setCursor(8, 0); // Définir le curseur à la position (0,0)
  lcd_screen.print(Vy_lcd); 
  // Print arming status
  lcd_screen.setCursor(15, 0); 
  if (is_Armed){ 
    lcd_screen.print("A"); 
  }
  else{
    lcd_screen.print("U"); 
  }

  // If omegaZ setpoint print it, else print current yaw and targets
  if (omegaZ_setpoint==0){ 
  String yaw_lcd = "Yaw:"+String(Mecanum_Car.get_yaw()).substring(0, 6);
  lcd_screen.setCursor(0, 1); // Définir le curseur à la position (0,0)
  lcd_screen.print(yaw_lcd); 
  String target_lcd = "T:"+String(Mecanum_Car.get_target_yaw()).substring(0, 6);
  lcd_screen.setCursor(9, 1); // Définir le curseur à la position (0,0)
  lcd_screen.print(target_lcd); 
  }
  else {
  String omegaZ_lcd = "Z:"+String(Mecanum_Car.get_omegaZ()).substring(0, 6);
  lcd_screen.setCursor(0, 1); // Définir le curseur à la position (0,0)
  lcd_screen.print(omegaZ_lcd);  
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
  if (LT){
    is_Armed = true;
  }
  else{
    is_Armed = false;
  }
  if (RT){ 
      if (!remote_controlled){   
      remote_controlled=true;   
      } 
  }
  else{
    if (remote_controlled){
      Mecanum_Car.reset_target_yaw();
      remote_controlled=false;
      }
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
  //remote_controlled="SERIAL";
  
  if (remote_controlled && is_Armed)
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
      Mecanum_Car.send_PWM(PWM1,PWM2,PWM3,PWM4);
      //Mecanum_Car.display_motor_speed();
      //Mecanum_Car.display_yaw();
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