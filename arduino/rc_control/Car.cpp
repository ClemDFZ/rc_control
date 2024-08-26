#include <Arduino.h>
#include "Car.h"

Car::Car(unsigned long SAMPLING_PERIOD ) :
	_FrontLeft_motor(52, 53, 4, 62, 63,SAMPLING_PERIOD),    // A8 -> 62, A9 -> 63
	_FrontRight_motor(50, 51, 5, 64, 65,SAMPLING_PERIOD),   // A10 -> 64, A11 -> 65
	_RearLeft_motor(26, 27, 6, 66, 67,SAMPLING_PERIOD),     // A12 -> 66, A13 -> 67
	_RearRight_motor(24, 25, 7, 68, 69,SAMPLING_PERIOD)   // A14 -> 68, A15 -> 69
	{
	_motors_list[0] = &_FrontLeft_motor;
	_motors_list[1] = &_FrontRight_motor;
	_motors_list[2] = &_RearLeft_motor;
	_motors_list[3] = &_RearRight_motor;
}

void Car::tachy_front_left()
{
  _FrontLeft_motor.increase_tachy();
}

void Car::tachy_front_right()
{
  _FrontRight_motor.increase_tachy();
}

void Car::tachy_rear_left()
{
  _RearLeft_motor.increase_tachy();
}

void Car::tachy_rear_right()
{
  _RearRight_motor.increase_tachy();
}

void Car::update_motor_PID()
{
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->PID_controller();
  }
}

void Car::update_motors_command()
{
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->send_PID_input();
    //motors_list[i]->sendPWM(-255);
  }
  /*
  for (int i=0;i<motors_list_length;i++)
  {
    motors_list[i]->sendPWM(255);
  }
  */
}

void Car::update_tachy()
{
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->update_rotation_speed();
  }
}

void Car::forward_kinematics(){
  // Calculer les vitesses des roues
  float omega1 = _FrontLeft_motor.getRotationSpeed();
  float omega2 = _FrontRight_motor.getRotationSpeed();
  float omega3 = _RearLeft_motor.getRotationSpeed();
  float omega4 = _RearRight_motor.getRotationSpeed();
  _Vx_odo = _wheel_radius*(omega1+omega2+omega3+omega4)/4;
  _Vy_odo = _wheel_radius*(-omega1+omega2+omega3-omega4)/4;
  _omegaZ_odo = _wheel_radius*(-omega1+omega2-omega3+omega4)/(4*(_L+_W));
}

void Car::set_car_setpoints(float Vx_setpoint,float Vy_setpoint,float omegaZ_setpoint){
	_Vx_setpoint = Vx_setpoint;
	_Vy_setpoint = Vy_setpoint;
	_omegaZ_setpoint = omegaZ_setpoint;
}

void Car::update_velocity_PID()
{       
    //Serial.println("=============");
    float d_Vx = _Vx_setpoint-_Vx_odo;
    _Vx_corrected = _Vx_setpoint+ _Kp_Vx*d_Vx;

    float d_Vy = _Vy_setpoint-_Vy_odo;
    _Vy_corrected = _Vy_setpoint+ _Kp_Vy*d_Vy;

    float d_omegaZ = _omegaZ_setpoint-_omegaZ_odo;
    _omegaZ_corrected = _omegaZ_setpoint+ _Kp_omegaZ*d_omegaZ;
}

void Car::set_motor_speed(float omega1,float omega2,float omega3,float omega4){ 
  float omega_list[4] {omega1,omega2,omega3,omega4};
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->setSpeed(omega_list[i]);
  }
}

void Car::inverse_kinematics(){
  // Calculer les vitesses des roues
  float omega1 = (_Vx_corrected - _Vy_corrected - (_L + _W) * _omegaZ_corrected)/_wheel_radius;
  float omega2 = (_Vx_corrected + _Vy_corrected + (_L + _W) * _omegaZ_corrected)/_wheel_radius;
  float omega3 = (_Vx_corrected + _Vy_corrected - (_L + _W) * _omegaZ_corrected)/_wheel_radius;
  float omega4 = (_Vx_corrected - _Vy_corrected + (_L + _W) * _omegaZ_corrected)/_wheel_radius;

  /*
  Serial.println();
  Serial.println(wheel_radius);
  Serial.println(Vx_corrected);
  Serial.println(Vy_corrected);
  Serial.println(omegaZ_corrected);
  Serial.println(omega1);
  */

  this->set_motor_speed(omega1,omega2,omega3,omega4);
}

void Car::display_fwd_kinematics(){
  /**/
  Serial.print(_Vx_setpoint);
  Serial.print(",");
  Serial.print(_Vx_corrected);
  Serial.print(",");
  Serial.print(_Vx_odo);
  Serial.println();
}

void Car::display_motor_speed(){
  /**/
  _FrontLeft_motor.print_commands();
}

void Car::RC_commands(){
	/*
  float Vx_setpoint = pitch;
  float Vy_setpoint = roll ; 
  float omegaZ_setpoint = yaw; 
  // Calcul des vitesses des roues
  float omega1 = (Vx_setpoint - Vy_setpoint - (_L + _W) * omegaZ_setpoint);
  float omega2 = (Vx_setpoint + Vy_setpoint + (_L + _W) * omegaZ_setpoint);
  float omega3 = (Vx_setpoint + Vy_setpoint - (_L + _W) * omegaZ_setpoint);
  float omega4 = (Vx_setpoint - Vy_setpoint + (_L + _W) * omegaZ_setpoint);
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
  this->set_motor_speed(omega1,omega2,omega3,omega4);
  */
}