#include <Arduino.h>
//#include "MPU_handler.h"

class MPU_handler{
  
  public:
    float get_last_measure();
    void update_measure(float measure){
      _last_measure = measure;
      if ((_yaw_meas_cpt)<_YAW_MEAS_AVG_MAX){
      _yaw_speed_list[_yaw_meas_cpt] = _last_measure;
      _yaw_meas_cpt+=1;
      }
    }
 
    float get_averaged_yaw()
    {
      float avg_yaw = 0.0;
      for (int x=0;x<_yaw_meas_cpt;x++)
      {	
        float meas_yaw = _yaw_speed_list[x];
        avg_yaw+=meas_yaw;
        //Serial.print(meas_yaw);
        //Serial.print(",");
        _yaw_speed_list[x] = 0.0;
      }

    avg_yaw = avg_yaw/(float)_yaw_meas_cpt;
    //Serial.println(avg_yaw);
    _yaw_meas_cpt = 0;

    return avg_yaw;
	}

    float get_averaged_speed()
    {
        float yaw = get_averaged_yaw();
        unsigned long now = micros();
        unsigned long dT = now-_last_measure_time;
        float dYaw = yaw-_last_avg_yaw;
        float speed = dYaw/dT*1000000.0*(PI/180);
        /*
        Serial.print(dYaw);
        Serial.print(",");
        Serial.print(dT);
        Serial.print(",");
        Serial.println(speed);
        */
        _last_avg_yaw = yaw;
        _last_measure_time = now;
        return speed;
    }



  private:
    static const unsigned int _YAW_MEAS_AVG_MAX=10;
    float _yaw_speed_list[_YAW_MEAS_AVG_MAX];
    unsigned int _yaw_meas_cpt=0;
    float _last_measure=0;
    unsigned long _last_measure_time = 0;
    float _last_avg_yaw = 0;
};