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
        Serial.print(meas_yaw);
        Serial.print(",");
        _yaw_speed_list[x] = 0.0;
      }

    avg_yaw = avg_yaw/(float)_yaw_meas_cpt;
    Serial.println(avg_yaw);
    _yaw_meas_cpt = 0;
    return avg_yaw;
	}

  private:
    static const unsigned int _YAW_MEAS_AVG_MAX=10;
    float _yaw_speed_list[_YAW_MEAS_AVG_MAX];
    unsigned int _yaw_meas_cpt=0;
    float _last_measure=0;
};