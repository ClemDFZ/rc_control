#ifndef CAR_H
#define CAR_H
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Motor.h"
#include "MPU_handler.cpp"
class Car {
    
public:
    Car(MPU_handler* mpu_handler,LiquidCrystal_I2C* lcd_screen,unsigned long SAMPLING_PERIOD);

    void tachy_front_left();
    void tachy_front_right();
    void tachy_rear_left();
    void tachy_rear_right();

    void update_motor_PID();

    void reset_target_yaw();

    void set_motor_Kp(float Kp);
    void set_motor_Ki(float Kd);
    void set_motor_Kd(float Ki);

    float get_target_yaw(){return _target_yaw;}
    float get_yaw(){return _yaw;}
    void update_motors_command();

    void send_PWM(float PWM);
    void send_PWM(float PWM1,float PWM2,float PWM3,float PWM4);

    void update_tachy();
    void inverse_kinematics();
    void forward_kinematics();
    void display_fwd_kinematics();
    void display_motor_speed();
    void update_velocity_PID();

    void set_Ki(float Ki){_Ki_Vx = Ki;}

    void set_Kp_Vx(float  Kp){_Kp_Vx = Kp;}  
    void set_Kp_yaw(float  Kp){_Kp_yaw = Kp;}  

    void set_car_setpoints(float Vx_setpoint,float Vy_setpoint,float omegaZ_setpoint);
    void set_motor_speed(float omega1,float omega2,float omega3,float omega4);
    void RC_commands();

private:
    // Distances inter-moteurs
    float _L = 0.210; // wheels length distance
    float _W = 0.210;  // wheels width distance
    float _wheel_radius=(97.0/2.0)/1000.0; //wheel radius (meter)

    float _Vx_odo,_Vy_odo,_omegaZ_odo;
    float _Vx_corrected,_Vy_corrected,_omegaZ_corrected;
    float _Vx_setpoint,_Vy_setpoint,_omegaZ_setpoint;	
    
    Motor _FrontLeft_motor,_FrontRight_motor,_RearLeft_motor,_RearRight_motor;
    Motor* _motors_list[4];
    int _motors_list_length = 4;
    
    MPU_handler* _mpu_handler;
    LiquidCrystal_I2C* _lcd_screen;
    float _yaw=0.0;
    float _target_yaw = 0.0;
    float _Kp_yaw = 0.05;
    float _Kp_Vx = 0.0;
    float _Ki_Vx = 0.00;
    float _Kp_Vy = 0.0;
    float _Ki_Vy = 0.00;
    float _Kp_omegaZ = 0.0;
    float _Ki_omegaZ = 0.000;
};

#endif // CAR_H