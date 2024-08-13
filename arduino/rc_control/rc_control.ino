
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

// Gestion fréquence boucle
const unsigned long SAMPLING_FREQUENCY = 50; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long SAMPLING_PERIOD = 1000 / SAMPLING_FREQUENCY; // Période en millisecondes

const unsigned long PID_FREQUENCY = 25; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long PID_PERIOD = 1000 / PID_FREQUENCY; // Période en millisecondes
unsigned long lastPIDTime = 0;

const unsigned long SETPOINT_FREQUENCY = 1; // Fréquence en Hz (nombre de boucles par seconde)
const unsigned long SETPOINT_PERIOD = 1000 / SETPOINT_FREQUENCY; // Période en millisecondes
unsigned long lastSetpointTime = 0;
float lastSetpoint = 2;
const float HIGH_SETPOINT = -1.0;
const float LOW_SETPOINT = 2.0;


class Motor {
public:
    // Constructeur pour initialiser les broches du moteur et des encodeurs
    Motor(int pinDirection1, int pinDirection2, int pinPWM, int pinEncoderA, int pinEncoderB)
        : _pinDirection1(pinDirection1), _pinDirection2(pinDirection2), _pinPWM(pinPWM), _pinEncoderA(pinEncoderA), _pinEncoderB(pinEncoderB) {
        // Configurer les broches de direction, PWM, et encodeurs
        pinMode(_pinDirection1, OUTPUT);
        pinMode(_pinDirection2, OUTPUT);
        pinMode(_pinPWM, OUTPUT);
        pinMode(_pinEncoderA, INPUT_PULLUP);
        pinMode(_pinEncoderB, INPUT_PULLUP);
    }
    void increase_tachy(){
        int currentEncoderA = digitalRead(_pinEncoderA);
        int currentEncoderB = digitalRead(_pinEncoderB);
        if (currentEncoderA != _lastEncoderA) {
            if (currentEncoderB != currentEncoderA) {
                //Serial.println("+");
                _tachy++;
            } else {
                //Serial.println("-");
                _tachy--;
            }
        }
        _lastEncoderA = currentEncoderA;     
    }

    void setSpeed(float target_rotation_speed)
    {
      _target_rotation_speed = target_rotation_speed ;
    }

    void PID_controller()
    {       
        //Serial.println("=============");
        float d_rpm = _target_rotation_speed-_rotationSpeed;
        float derivate_d_rpm = d_rpm-_previous_error;
        Serial.print(_target_rotation_speed);
        Serial.print(",");
        Serial.println(_rotationSpeed);
        //Serial.println(d_rpm);
        _sum_d_rpm += d_rpm;
        float I_sat = constrain(_sum_d_rpm*_Ki,-20,20);
        _pwm_corrected = (_target_rotation_speed/_top_rotation_speed)*255+d_rpm*_Kp + I_sat +_Kd*derivate_d_rpm;
        _previous_error = d_rpm;
    }

    void send_PID_input()
    {
      sendPWM(_pwm_corrected);
    }

    void set_Ki(float Ki){
      _Ki = Ki;
    }

    void set_Kp(float  Kp){
      _Kp = Kp;
    }
    void set_Kd(float  Kd){
      _Kd = Kd;
    }
    // Méthode pour régler la vitesse et la direction du moteur
    void sendPWM(int speed) {
      if (speed != 0) {
        if (speed >= 0) {
            // Sens de rotation horaire
            digitalWrite(_pinDirection1, LOW);
            digitalWrite(_pinDirection2, HIGH);
        } else {
            // Sens de rotation antihoraire
            digitalWrite(_pinDirection1, HIGH);
            digitalWrite(_pinDirection2, LOW);
            speed = -speed;  // Convertir la vitesse en valeur positive pour PWM
        }
        analogWrite(_pinPWM, constrain(speed, 0, 255));
        }
    }

    // Méthode pour mettre à jour la rotation basée sur l'encodeur
    void update_rotation_speed() {
        //Serial.println(_tachy);
        float nb_tour = (float)_tachy/(float)tachy_per_turn;
        float round_per_ms = nb_tour/SAMPLING_PERIOD*1000;
        _rotationSpeed = round_per_ms;   
        _tachy = 0; // Exemple simplifié, ici _rotationSpeed est basé sur les impulsions accumulées
    }

    // Obtenir la vitesse de rotation actuelle
    float getRotationSpeed() {
        return _rotationSpeed;
    }

private:
    const int tachy_per_turn = 816;
    const float _top_rotation_speed = 5.5;
    int _pinDirection1;
    int _pinDirection2;
    int _pinPWM;
    int _pinEncoderA;
    int _pinEncoderB;

    int _pwm_corrected;
    float _Kp = 20;
    float _Ki = 0.5;
    float _Kd = 0;
    float _sum_d_rpm = 0;
    float _previous_error = 0;
    float _target_rotation_speed = 0;
    int _tachy = 0;
    volatile int _lastEncoderA = LOW;
    float _rotationSpeed = 0.0;
};


// Création moteurs et attribution PIN
Motor FrontLeft_motor(52, 53, 4, 62, 63);    // A8 -> 62, A9 -> 63
Motor FrontRight_motor(50, 51, 5, 64, 65);   // A10 -> 64, A11 -> 65
Motor RearLeft_motor(26, 27, 6, 66, 67);     // A12 -> 66, A13 -> 67
Motor RearRight_motor(24, 25, 7, 68, 69);    // A14 -> 68, A15 -> 69



// Gestion RC
void RC_callback();
int throttle,roll,pitch,yaw,LB,RB,LP;
bool RT = true;
bool LT = true;

String control_mode = "SERIAL";
String receivedData="";
float X_command = 0.0;
float Y_command = 0.0;
float Z_command = 0.0;
float Vx,Vy,dtheta;


/**
 * @brief Executed once on power-up or reboot
 */
void setup() {
  Serial.begin(115200);

  while (!Serial) {
      ; // Attendre que la connexion série soit établie (utile lors de l'utilisation de certaines cartes comme l'UNO)
    }

  crsf.begin();
  crsf.onPacketChannels = &RC_callback;

  // Initialiser la pin d'interruption pour Encoder A
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(62), tachy_front_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(64), tachy_front_right, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(66), tachy_rear_left, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(68), tachy_rear_right, CHANGE);

}

/**
 * @brief Executes cyclically while the power is on
 */

void loop() { 
  unsigned long t0 = millis();
  crsf.loop(); 
  readSerial();
  if(control_mode == "RC") {
    RC_commands();
  } else if (control_mode=="SERIAL")
  {
    kinematics();
  }
  update_sensors();

  unsigned long now = millis();
    // Vérifier si l'intervalle PID est atteint
  if (now - lastPIDTime >= PID_PERIOD) {
      lastPIDTime = now; // Mettre à jour le dernier temps PID
      // Calcul du PID
      update_motor_PID();
  }
  if (now - lastSetpointTime >= SETPOINT_PERIOD) {
    if (lastSetpoint==LOW_SETPOINT)
    {
      lastSetpoint = HIGH_SETPOINT;
    }
    else{
      lastSetpoint = LOW_SETPOINT;
    }
    lastSetpointTime = now;
    FrontLeft_motor.setSpeed(lastSetpoint);
  }

  
  update_motors_command();
  unsigned long deltaT = millis()-t0;
  if (deltaT < SAMPLING_PERIOD) {
    delay(SAMPLING_PERIOD - deltaT);
  }
  }

void update_motors_command() {
    //FrontLeft_motor.sendPWM(255);
    FrontLeft_motor.send_PID_input();
    //FrontRight_motor.send_PID_input();
    //RearLeft_motor.send_PID_input();
    //RearRight_motor.send_PID_input(); 
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

void update_motor_PID()
{
  FrontLeft_motor.PID_controller();
    //FrontRight_motor.PID_controller();
    //RearLeft_motor.PID_controller();
    //RearRight_motor.PID_controller(); 
}

void update_sensors()
{
    FrontLeft_motor.update_rotation_speed();
    //FrontRight_motor.update_rotation_speed();
    //RearLeft_motor.update_rotation_speed();
    //RearRight_motor.update_rotation_speed(); 
}

void readSerial()
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
    X_command = str.substring(xIndex + 1, spaceIndex).toFloat();
    FrontLeft_motor.setSpeed(X_command);
  }

  if (yIndex != -1) {
    int spaceIndex = str.indexOf(' ', yIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Y' est le dernier élément
    Y_command = str.substring(yIndex + 1, spaceIndex).toFloat();
  }

  if (zIndex != -1) {
    int spaceIndex = str.indexOf(' ', zIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    Z_command = str.substring(zIndex + 1, spaceIndex).toFloat();
  }

  if (pIndex != -1) {
    int spaceIndex = str.indexOf(' ', pIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Kp = str.substring(pIndex + 1, spaceIndex).toFloat();
    FrontLeft_motor.set_Kp(Kp);
  }

  if (iIndex != -1) {
    int spaceIndex = str.indexOf(' ', iIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Ki = str.substring(iIndex + 1, spaceIndex).toFloat();
    FrontLeft_motor.set_Ki(Ki);
  }

  if (dIndex != -1) {
    int spaceIndex = str.indexOf(' ', dIndex);
    if (spaceIndex == -1) spaceIndex = str.length(); // Cas où 'Z' est le dernier élément
    float Kd = str.substring(dIndex + 1, spaceIndex).toFloat();
    FrontLeft_motor.set_Kd(Kd);
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
  Vx = pitch;
  Vy = roll ; 
  dtheta = yaw; 
  // Calcul des vitesses des roues
  float omega1 = (Vx - Vy - (L + W) * dtheta);
  float omega2 = (Vx + Vy + (L + W) * dtheta);
  float omega3 = (Vx + Vy - (L + W) * dtheta);
  float omega4 = (Vx - Vy + (L + W) * dtheta);
  float maxSpeed = max(max(abs(omega1), abs(omega2)), max(abs(omega3), abs(omega4)));
  float maxThrottle = max((float)max(abs(Vy),(float)abs(Vx)),(float)abs(dtheta))/100;
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

void kinematics(){
  Vx     = X_command;
  Vy     = Y_command;
  dtheta = Z_command;
  // Calculer les vitesses des roues
  float omega1 = (Vx - Vy - (L + W) * dtheta);
  float omega2 = (Vx + Vy + (L + W) * dtheta);
  float omega3 = (Vx + Vy - (L + W) * dtheta);
  float omega4 = (Vx - Vy + (L + W) * dtheta);
  update_commands(omega1,omega2,omega3,omega4);
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