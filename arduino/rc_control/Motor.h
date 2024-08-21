#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    // Constructeur pour initialiser les broches du moteur et des encodeurs
    Motor(int pinDirection1, int pinDirection2, int pinPWM, int pinEncoderA, int pinEncoderB,unsigned long sampling_period);
    void increase_tachy();
    void setSpeed(float target_rotation_speed) {_target_rotation_speed = target_rotation_speed;}
    void PID_controller();

    void get_rev_counter();
    void print_commands();

    void send_PID_input();

    void set_Ki(float Ki){_Ki = Ki;}

    void set_Kp(float  Kp){_Kp = Kp;}
    
    void set_Kd(float  Kd){_Kd = Kd;}

    // Méthode pour régler la vitesse et la direction du moteur
    void sendPWM(int speed);

    // Méthode pour mettre à jour la rotation basée sur l'encodeur
    void update_rotation_speed();
    // Obtenir la vitesse de rotation actuelle
    float getRotationSpeed();
private:
    unsigned long _SAMPLING_PERIOD;
    const int tachy_per_turn = 816; //Résolution encodeur
    const float _top_rotation_speed = 5.5*2*PI; // Vmax moteur, à changer en dynamique dépendemment voltage
    const int _MAX_PWM_DIFF = 50;
    int _tachy = 0;
    long int _total_tachy = 0;
    float _rotationSpeed = 0.0;

    //Pins attribuées au moteur
    int _pinDirection1;
    int _pinDirection2;
    int _pinPWM;
    int _pinEncoderA;
    int _pinEncoderB;

    //PID values
    float _target_rotation_speed = 0;
    int _pwm_corrected;
    int _previous_pwm = 0;
    float _Kp = 20;
    float _Ki = 3;
    float _Kd = 10;
    float _sum_d_rpm = 0;
    float _previous_error = 0;
    
    // Status pin interrupteur encodeur
    volatile int _lastEncoderA = LOW;
    
};

#endif // MOTOR_H