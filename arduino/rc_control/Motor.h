#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    // Constructeur pour initialiser les broches du moteur et des encodeurs
    Motor(int pinDirection1, int pinDirection2, int pinPWM, int pinEncoderA, int pinEncoderB);
    void increase_tachy();
    void setSpeed(float target_rotation_speed) {_target_rotation_speed = target_rotation_speed;}
    void PID_controller();

    void get_rev_counter();
    void print_commands();

    void send_PID_input();
    
    void set_Ki(float Ki){_Ki = Ki;}

    void set_Kp(float  Kp){_Kp = Kp;}
    
    void set_Kd(float  Kd){_Kd = Kd;}

    void set_buffer_size(int buffer_size){_PWM_BUFFER_SIZE=buffer_size;}
    void set_max_pwm(int max_pwm){_MAX_PWM_DIFF=max_pwm;}

    // Méthode pour régler la vitesse et la direction du moteur
    int filterPWM(int PWM);
    void sendPWM(int PWM);

    // Méthode pour mettre à jour la rotation basée sur l'encodeur
    void update_rotation_speed();
    // Obtenir la vitesse de rotation actuelle
    float getRotationSpeed();
    float get_averaged_speed();

private:
    float linear_pwm_command(float target_radpersec);
    const int tachy_per_turn = 816; //Résolution encodeur
    const float _top_rotation_speed = 5.5*2*PI; // Vmax moteur, à changer en dynamique dépendemment voltage
    int _tachy = 0;
    long int _total_tachy = 0;
    float _rotationSpeed = 0.0;

    //Pins attribuées au moteur
    int _pinDirection1;
    int _pinDirection2;
    int _pinPWM;
    int _pinEncoderA;
    int _pinEncoderB;

    static const int _SPEED_MEAS_AVG_MAX = 10;
    float _rot_speed_list[_SPEED_MEAS_AVG_MAX];
    unsigned int _rot_meas_cpt = 0;

    int _PWM_BUFFER_SIZE = 10;
    int _pwm_buffer[2];
    int _MAX_PWM_DIFF = 40;
    int _buffer_index = 0;

    //PID values
    unsigned long _last_t_tachy = micros();
    float _tachy_to_rpm_calculus = 7.699982;
    float _target_rotation_speed = 0;
    int _pwm_corrected;
    float _previous_pwm = 0;
    float _Kp = 9;
    float _Ki = 0.1;
    float _Kd = 3;
    float _sum_d_rpm = 0;
    float _previous_error = 0;
    float _last_avg_target = 0;
    float _pwm_sum ;
    int _pwm_count;
    float _error_threshold = 1.5;
    const int _avg_window_size = 5;
    // Status pin interrupteur encodeur
    volatile int _lastEncoderA = LOW;
    
};

#endif // MOTOR_H