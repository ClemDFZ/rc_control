#include <Arduino.h>
#include "Motor.h"

// Constructeur pour initialiser les broches du moteur et des encodeurs
Motor::Motor(int pinDirection1, int pinDirection2, int pinPWM, int pinEncoderA, int pinEncoderB,unsigned long sampling_period)
        : _pinDirection1(pinDirection1), _pinDirection2(pinDirection2), _pinPWM(pinPWM), _pinEncoderA(pinEncoderA), _pinEncoderB(pinEncoderB) {
        // Configurer les broches de direction, PWM, et encodeurs
        pinMode(_pinDirection1, OUTPUT);
        pinMode(_pinDirection2, OUTPUT);
        pinMode(_pinPWM, OUTPUT);
        pinMode(_pinEncoderA, INPUT_PULLUP);
        pinMode(_pinEncoderB, INPUT_PULLUP);
		_SAMPLING_PERIOD = sampling_period;
    }

void Motor::increase_tachy(){
	int currentEncoderA = digitalRead(_pinEncoderA);
	int currentEncoderB = digitalRead(_pinEncoderB);
	if (currentEncoderA != _lastEncoderA) {
		if (currentEncoderB != currentEncoderA) {
			//Serial.println("+");
			_tachy++;
			_total_tachy++;
		} else {
				//Serial.println("-");
			_tachy--;
			_total_tachy--;
		}
		//Serial.println(_total_tachy);
	}
	_lastEncoderA = currentEncoderA;     
}

void Motor::PID_controller()
{       
	//Serial.println("=============");




	float d_rpm = _target_rotation_speed-_rotationSpeed;
	float derivate_d_rpm = d_rpm-_previous_error;
	if (!isnan(d_rpm)) {
		_sum_d_rpm += d_rpm;
	}

	if ((d_rpm > 0 && _previous_error < 0) || (d_rpm < 0 && _previous_error > 0)) {
    	_sum_d_rpm = 0;}

	//float I_sat = _sum_d_rpm*_Ki;
	// Si le régime permanent est atteint (erreur stable et faible)
	float target_pwm = linear_pwm_command(_target_rotation_speed);
			
	/*
	if (_last_avg_target != 0){float target_pwm=_last_avg_target;}
	else{}
	// Calculer la moyenne après un certain nombre de cycles
	if ((_pwm_count >= _avg_window_size) && (_last_avg_target != 0)){
		Serial.print("Estimate: ");
		Serial.println(_pwm_sum / _pwm_count);
		_last_avg_target = _pwm_sum / _pwm_count;
		_pwm_sum = 0;
		_pwm_count = 0;
	}
	else{
		if (abs(d_rpm) < _error_threshold) {
				// Ajouter la consigne à la moyenne
				_pwm_sum += _pwm_corrected;
				_pwm_count++;
			} 
		else{
			//_pwm_sum = 0;
			//_pwm_count = 0;
		}
	}
	*/
	
	//float target_pwm = linear_pwm_command(_target_rotation_speed);
	float P = d_rpm*_Kp;
	float I = constrain(_Ki*_sum_d_rpm,-50,50);
	float D = -_Kd*derivate_d_rpm;

	_pwm_corrected = target_pwm+P+I+D;
	_pwm_corrected = constrain(_pwm_corrected, -255, 255);


	/* 
	Serial.print(P);
	Serial.print(",");
	Serial.print(I);
	Serial.print(",");	
	Serial.print(D);
	Serial.print(",");
	
	Serial.print(target_pwm);
	Serial.print(",");
	Serial.print(_pwm_corrected);
	Serial.print(",");
	Serial.println();
	*/
	_previous_error = d_rpm;
	//print_commands();
}

void Motor::get_rev_counter()
{
	float total_dist = (float)_total_tachy/(float)tachy_per_turn*2*PI;
	return total_dist;
}

void Motor::print_commands()
{
	Serial.print(_target_rotation_speed);
	Serial.print(",");
	Serial.println(_rotationSpeed);
}

float Motor::linear_pwm_command(float target_radpersec){
	if (target_radpersec>=0){
	float target_pwm = 0.0013*pow(target_radpersec,4) - 0.0617*pow(target_radpersec,3) + 1.0092*pow(target_radpersec,2) - 3.3109*target_radpersec + 32.435;
	return target_pwm;
	}
	else{
		target_radpersec = -target_radpersec;
		float target_pwm = 0.0013*pow(target_radpersec,4) - 0.0617*pow(target_radpersec,3) + 1.0092*pow(target_radpersec,2) - 3.3109*target_radpersec + 32.435;
		return -target_pwm;
	}
}

void Motor::send_PID_input()
{
	if ((abs(_previous_pwm-_pwm_corrected))>_MAX_PWM_DIFF){
	}
	sendPWM(_pwm_corrected);
	_previous_pwm = _pwm_corrected;
}



// Méthode pour régler la vitesse et la direction du moteur
void Motor::sendPWM(int speed) {
	//if (speed != 0) {
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
	//}
}

// Méthode pour mettre à jour la rotation basée sur l'encodeur
void Motor::update_rotation_speed() {
	unsigned long now = micros();
	_rotationSpeed = (float)_tachy/(now-_last_t_tachy)*1000.0*_tachy_to_rpm_calculus;
	/*
	
	Serial.print(_rotationSpeed);
	Serial.print(",");
	Serial.println(_tachy);
	*/
	_last_t_tachy = now;
	_tachy = 0; // Tachy remise à 0

	if ((_rot_meas_cpt)<_SPEED_MEAS_AVG_MAX){
		_rot_speed_list[_rot_meas_cpt] = _rotationSpeed;
		_rot_meas_cpt+=1;
	}
	
}

// Obtenir la vitesse de rotation actuelle
float Motor::getRotationSpeed() {
	return _rotationSpeed;
}


float Motor::get_averaged_speed(){
	float avg_speed;
	if (_rot_meas_cpt!=0){
	for (int x=0;x<_rot_meas_cpt;x++)
	{	
		float meas_speed = _rot_speed_list[x];
		avg_speed+=meas_speed;
		//Serial.print(meas_speed);
		//Serial.print(",");
		_rot_speed_list[x] = 0.0;
	}
	
	avg_speed = avg_speed/(float)_rot_meas_cpt;
	}
	else{
	avg_speed = _rotationSpeed;
	}
	//Serial.println(avg_speed);
	_rot_meas_cpt = 0;
	return avg_speed;
	}
	
