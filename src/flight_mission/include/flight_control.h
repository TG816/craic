#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "mission_header.h"
#include "ring_detection.h"


/************************************************************************
飞控控制函数声明
*************************************************************************/
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool precision_land();
bool move_in_drone_coordinate(double x, double y, double z, double target_yaw, double err_max, int mode = 0);
void fly(float v);
bool Circle_around(int counts, float times, double err_max);
bool Circle_around(int counts, float times, float z_h, float v0, float v1, float cx, float cy, float r_of_c, float err_max);
bool execute_universal_crossing(float err_max);
bool lib_time_record_func(float duration, ros::Time current_time);
bool hover(float time_duration);

#endif