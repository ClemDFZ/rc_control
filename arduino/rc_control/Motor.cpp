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
	_sum_d_rpm += d_rpm;
	//float I_sat = _sum_d_rpm*_Ki;
	float I_sat = constrain(_sum_d_rpm*_Ki,-100,100);
	_pwm_corrected = (_target_rotation_speed/_top_rotation_speed)*255+d_rpm*_Kp + I_sat +_Kd*derivate_d_rpm;
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

void Motor::send_PID_input()
{
	if ((abs(_previous_pwm-_pwm_corrected))>_MAX_PWM_DIFF){
	}
	sendPWM(_pwm_corrected);
	_previous_pwm = _pwm_corrected;
}

// Méthode pour régler la vitesse et la direction du moteur
void Motor::sendPWM(int speed) {
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
void Motor::update_rotation_speed() {
	float nb_tour = (float)_tachy/(float)tachy_per_turn;
	float round_per_sec = nb_tour/_SAMPLING_PERIOD*1000;
	float rad_per_sec = round_per_sec*2*PI;
	_rotationSpeed = rad_per_sec;  
	/*
	Serial.println(); 
	Serial.println(_rotationSpeed);
	Serial.println(_tachy);
	Serial.println(_pinDirection1);
	*/
	_tachy = 0; // Tachy remise à 0
}

// Obtenir la vitesse de rotation actuelle
float Motor::getRotationSpeed() {
	return _rotationSpeed;
}