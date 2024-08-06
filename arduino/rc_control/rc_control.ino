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

#if defined(__AVR_ATmega2560__)
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);
#else
#error NOT MEGA2560
#endif

class Motor {
public:
    // Constructeur pour initialiser les broches du moteur
    Motor(int pinDirection1, int pinDirection2, int pinPWM) 
      : _pinDirection1(pinDirection1), _pinDirection2(pinDirection2), _pinPWM(pinPWM) {
        pinMode(_pinDirection1, OUTPUT);
        pinMode(_pinDirection2, OUTPUT);
        pinMode(_pinPWM, OUTPUT);
    }

    // Méthode pour régler la vitesse et la direction du moteur
    void setSpeed(int speed) {
        // Déterminer le sens de rotation
        if (speed >= 0) {
            // Sens de rotation horaire
            digitalWrite(_pinDirection1, LOW);
            digitalWrite(_pinDirection2, HIGH);
        } else {
            // Sens de rotation antihoraire
            digitalWrite(_pinDirection1, HIGH);
            digitalWrite(_pinDirection2, LOW);
            speed = -speed; // Convertir la vitesse en valeur positive pour PWM
        }
        // Régler la vitesse du moteur
        analogWrite(_pinPWM, constrain(speed, 0, 255));
    }

private:
    int _pinDirection1; // Broche pour la direction (sens 1)
    int _pinDirection2; // Broche pour la direction (sens 2)
    int _pinPWM;        // Broche pour la vitesse PWM
};

void RC_callback();
int throttle,roll,pitch,yaw,LT,RT,LB,RB,LP;
Motor FrontLeft_motor(52,53,4);
Motor FrontRight_motor(50,51,5);
Motor RearLeft_motor(26,27,6);
Motor RearRight_motor(24,25,7);

int PrintCPT = 0;

/**
 * @brief Executed once on power-up or reboot
 */
void setup() {
  Serial.begin(115200);
  crsf.begin();
  crsf.onPacketChannels = &RC_callback;
}

/**
 * @brief Executes cyclically while the power is on
 */
void loop() { 
  crsf.loop(); 
  Serial.print("Running");
  Serial.println(); 
  }

void RC_callback() {
  PrintCPT++;
  refresh_RC_commands();
  kinematics();
  /*
  if (PrintCPT> 100){
    PrintCPT = 0;
    kinematics();
  }
  */
  
}

void refresh_RC_commands(){
  //for (auto ch{1}; ch <= CRSF_NUM_CHANNELS; ++ch) {
  throttle = crsf.getChannel(1);
  throttle = constrain(map(throttle,1000,2000,-100,100), -100, 100);
  roll = crsf.getChannel(2);
  roll = -constrain(map(roll,1000,2000,-100,100), -100, 100);
  pitch= crsf.getChannel(3);
  pitch = constrain(map(pitch,1000,2000,-100,100), -100, 100);
  yaw= crsf.getChannel(4);
  yaw = -constrain(map(yaw,1000,2000,-100,100), -100, 100);  
  LT= single_switch(crsf.getChannel(5));
  RT= single_switch(crsf.getChannel(6));
  LB= double_switch(crsf.getChannel(7));
  RB= double_switch(crsf.getChannel(8));
  LP= crsf.getChannel(9);  
  //}

}

void move_motors(){

}
float L = 0.25; // wheels length distance
float W = 0.2;  // wheels width distance

void kinematics(){
  // Calculer les vitesses des roues
  float omega1 = (pitch - roll - (L + W) * yaw);
  float omega2 = (pitch + roll + (L + W) * yaw);
  float omega3 = (pitch + roll - (L + W) * yaw);
  float omega4 = (pitch - roll + (L + W) * yaw);
  float maxSpeed = max(max(abs(omega1), abs(omega2)), max(abs(omega3), abs(omega4)));
  float maxThrottle = max((float)max(abs(roll),(float)abs(pitch)),(float)abs(yaw))/100;
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


  int pwmFL = constrain(omega1,-255,255); // Convertir float en int
  int pwmFR = constrain(omega2,-255,255); // Convertir float en int
  int pwmRL = constrain(omega3,-255,255); // Convertir float en int
  int pwmRR = constrain(omega4,-255,255); // Convertir float en int

  Serial.print("====================");
  Serial.println(); 
  Serial.print(pitch);
  Serial.println();
  Serial.print(roll);
  Serial.println();
  
  Serial.print("OMEGA");
  Serial.println();

  Serial.print(omega1);
  Serial.println();
  Serial.print(omega2);
  Serial.println();  
  Serial.print(omega3);
  Serial.println();
  Serial.print(omega4);
  Serial.println();

  Serial.print("THROTTLE");
  Serial.println();
  Serial.print(maxSpeed);
  Serial.println();  
  Serial.print(maxThrottle);
  Serial.println();

  Serial.print("PWM");
  Serial.println();
  Serial.print(pwmFL);
  Serial.println();  
  Serial.print(pwmFR);
  Serial.println();

  FrontLeft_motor.setSpeed(pwmFL);
  FrontRight_motor.setSpeed(pwmFR);
  RearLeft_motor.setSpeed(pwmRL);
  RearRight_motor.setSpeed(pwmRR);
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