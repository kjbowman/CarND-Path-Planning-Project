#ifndef COST_H
#define COST_H

#include "vehicle.h"


float inefficiency_cost(double target_speed, int intended_lane, int final_lane,
                          const vector<double>& lane_speeds);

float collision_cost(double ego_s, int ego_lane, double ego_speed,
                     int desired_lane, vector<Vehicle> tracked_objects);

float congestion_cost(const Vehicle& ego, int lane,
                       const vector<Vehicle> tracked_objects);



#endif // COST_H