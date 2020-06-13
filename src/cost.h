#ifndef COST_H
#define COST_H

#include "vehicle.h"


float calculate_cost(const Vehicle& vehicle,
                     const vector<Vehicle>& predictions,
                     const vector<Vehicle>& trajectory);

float inefficiency_cost(double target_speed, int intended_lane, int final_lane,
                          const vector<double>& lane_speeds);

float collision_cost(double ego_s, int ego_lane, double ego_speed,
                     int desired_lane, vector<Vehicle> tracked_objects);

float congestion_cost(double ego_s, int lane,
                       const vector<Vehicle> tracked_objects);



#endif // COST_H