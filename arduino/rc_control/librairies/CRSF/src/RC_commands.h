/**
 * @file RC_commands.h
 * @author ClemDFZ
 * @brief
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef RC_COMMANDS_H
#define RC_COMMANDS_H

class RC_commands 
{
  public:
    void update_values(const int throttle,const int roll,const int pitch,const int yaw,const int LT,const int RT,const int LB,const int RB, const int LP);

  private:
    int _throttle;
    int _roll;
    int _pitch;
    int _yaw;
    int _LT;
    int _RT;
    int _LB;
    int _RB;
    int _LP;
};


#endif