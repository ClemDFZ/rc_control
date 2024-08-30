#include <Arduino.h>
#include "Car.h"



Car::Car(MPU_handler* mpu_handler,unsigned long SAMPLING_PERIOD ) :
	_FrontLeft_motor(50, 51, 5, 62, 63,SAMPLING_PERIOD),    // A8 -> 62, A9 -> 63
	_FrontRight_motor(48, 49, 6, 64, 65,SAMPLING_PERIOD),   // A10 -> 64, A11 -> 65
	_RearLeft_motor(26, 27, 7, 66, 67,SAMPLING_PERIOD),     // A12 -> 66, A13 -> 67
	_RearRight_motor(24, 25, 8, 68, 69,SAMPLING_PERIOD),  // A14 -> 68, A15 -> 69
  _mpu_handler(mpu_handler)
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
/*
  _FrontLeft_motor.PID_controller();
  */
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->PID_controller();
  }
  
}

void Car::send_PWM(float PWM)
{
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->sendPWM(PWM);
  }
}

void Car::send_PWM(float PWM1,float PWM2,float PWM3,float PWM4)
{
  float pwm_list[4] {PWM1,PWM2,PWM3,PWM4};
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->sendPWM(pwm_list[i]);
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
  float omega1 = _FrontLeft_motor.get_averaged_speed();
  float omega2 = _FrontRight_motor.get_averaged_speed();
  float omega3 = _RearLeft_motor.get_averaged_speed();
  float omega4 = _RearRight_motor.get_averaged_speed();
  //Serial.print(omega1);
  //Serial.print(",");
  _Vx_odo = _wheel_radius*(omega1+omega2+omega3+omega4)/4;
  _Vy_odo = _wheel_radius*(-omega1+omega2+omega3-omega4)/4;
  //_omegaZ_odo = _wheel_radius*(-omega1+omega2-omega3+omega4)/(4*(_L+_W));
  _omegaZ_odo=-_mpu_handler->get_averaged_speed();
  _yaw = -_mpu_handler->get_last_measure();

  //Serial.println(_yaw);
  /**/
  Serial.print(_Vx_odo);
  Serial.print(",");
  Serial.print(_Vy_odo);
  Serial.print(",");
  Serial.print(_omegaZ_odo);
  Serial.print(",");
  Serial.println(_yaw);
  
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
   
    if (_omegaZ_setpoint!= 0){
    _target_yaw=0;
    float d_omegaZ = _omegaZ_setpoint-_omegaZ_odo;
    _omegaZ_corrected = _omegaZ_setpoint+ _Kp_omegaZ*d_omegaZ;
    }

    else{
      float d_yaw = _target_yaw-_yaw;
      _omegaZ_corrected = _Kp_yaw*d_yaw;
      //Serial.println(_omegaZ_corrected);
    }

/*  
    Serial.print(_Vx_setpoint);
    Serial.print(",");
    Serial.print(d_Vx);
    Serial.print(",");
    Serial.print(_Vx_corrected);
    Serial.print(",");

    Serial.print(d_omegaZ);
    Serial.print(",");
    Serial.println(_omegaZ_corrected);
  */ 
}

void Car::set_motor_Kp(float Kp){ 
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->set_Kp(Kp);
  }
}

void Car::set_motor_Ki(float Ki){ 
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->set_Ki(Ki);
  }
}

void Car::set_motor_Kd(float Kd){ 
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->set_Kd(Kd);
  }
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
  /*
  Serial.print(_Vx_corrected);
  Serial.print(",");
  Serial.print(_Vy_corrected);
  Serial.print(",");
  Serial.print(_omegaZ_corrected);
  Serial.println();
*/

  float omega1 = (_Vx_corrected - _Vy_corrected - (_L + _W) * _omegaZ_corrected)/_wheel_radius;
  float omega2 = (_Vx_corrected + _Vy_corrected + (_L + _W) * _omegaZ_corrected)/_wheel_radius;
  float omega3 = (_Vx_corrected + _Vy_corrected - (_L + _W) * _omegaZ_corrected)/_wheel_radius;
  float omega4 = (_Vx_corrected - _Vy_corrected + (_L + _W) * _omegaZ_corrected)/_wheel_radius;


/*
  Serial.print(omega1);
  Serial.print(",");
  Serial.print(omega2);
  Serial.print(",");
  Serial.print(omega3);
  Serial.print(",");
  Serial.print(omega4);
  Serial.println();
 */

  this->set_motor_speed(omega1,omega2,omega3,omega4);
}



void Car::display_fwd_kinematics(){
  /* */
  Serial.print(_Vx_setpoint);
  Serial.print(",");
  Serial.print(_Vx_corrected);
  Serial.print(",");
  Serial.print(_Vx_odo);
  Serial.println();
}

void Car::display_motor_speed(){
  /*
  _FrontLeft_motor.print_commands();
*/
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->print_commands();
  }
  Serial.println();


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