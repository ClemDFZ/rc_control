#include <Arduino.h>
#include "Car.h"



Car::Car(MPU_handler* mpu_handler) :
	_FrontLeft_motor(50, 51, 4, 62, 63),    // A8 -> 62, A9 -> 63
	_FrontRight_motor(48, 49, 5, 64, 65),   // A10 -> 64, A11 -> 65
	_RearLeft_motor(26, 27, 6, 66, 67),     // A12 -> 66, A13 -> 67
	_RearRight_motor(24, 25, 7, 68, 69),  // A14 -> 68, A15 -> 69
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
  _FrontLeft_motor.sendPWM(PWM);
  /*
  for (int i=0;i<_motors_list_length;i++)
  {
    _motors_list[i]->sendPWM(PWM);
  }
  */
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
  //_omegaZ_gyro = _wheel_radius*(-omega1+omega2-omega3+omega4)/(4*(_L+_W));
  _omegaZ_gyro=-_mpu_handler->get_averaged_speed();
  _yaw = -_mpu_handler->get_last_measure();

  //Serial.println(_yaw);
  /*
  Serial.print(_Vx_odo);
  Serial.print(",");
  Serial.print(_Vy_odo);
  Serial.print(",");
  Serial.print(_omegaZ_gyro);
  Serial.print(",");
  Serial.println(_yaw);
  */
}

void Car::reset_target_yaw(){
  //Serial.println(_target_yaw);
  _target_yaw = -_mpu_handler->get_last_measure();
  //Serial.println(_target_yaw);
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
    reset_target_yaw();
    float d_omegaZ = _omegaZ_setpoint-_omegaZ_gyro;
    _omegaZ_corrected = _omegaZ_setpoint+ _Kp_omegaZ*d_omegaZ+_Ki_omegaZ;
    }

    else{
      float d_yaw = _target_yaw-_yaw;

      if (!isnan(d_yaw)) {
        _sum_dyaw += d_yaw;
      }

      if ((_sum_dyaw > 0 && _previous_dyaw < 0) || (_sum_dyaw < 0 && _previous_dyaw > 0)) {
          _sum_dyaw = 0;}
      float I_term = constrain(_Ki_yaw*_sum_dyaw,-1.0,1.0);
      _omegaZ_corrected = _Kp_yaw*d_yaw+I_term;
      _previous_dyaw = d_yaw;
      /*
      Serial.print(_Vx_setpoint);
      Serial.print(",");
      Serial.print(d_Vx);
      Serial.print(",");
      Serial.print(_Vx_corrected);
      Serial.print(",");
      Serial.print(_target_yaw);
      Serial.print(",");
      Serial.print(_yaw);
      Serial.print(",");
      Serial.println(_omegaZ_corrected);
      */
    }

/*  
    Serial.print(_Vx_setpoint);
    Serial.print(",");
    Serial.print(d_Vx);
    Serial.print(",");
    Serial.print(_Vx_corrected);
    Serial.print(",");
*/ 
   
  
}

void Car::set_buffer_size(int buffer_size){
  for (int i=0;i<_motors_list_length;i++)
    {
      _motors_list[i]->set_buffer_size(buffer_size);
    }
    }

void Car::set_max_pwm(int max_pwm){
  for (int i=0;i<_motors_list_length;i++)
    {
      _motors_list[i]->set_max_pwm(max_pwm);
    }
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