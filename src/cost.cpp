#include <math.h>
#include <vector>
#include "parameters.h"
#include "cost.h"



// adapted from "Implement a Second Cost Function in C++" lesson
// target_speed: desired speed (i.e. near speed limit)
// intended_lane: intendend lane for the given behavior (PLCL/R, LCL/R => one lane over) 
// final_lane: immediate resulting lane of the behavior. LCL/R => one lane over
// lane_speeds: vector of lane speeds , based on sensed traffic
float inefficiency_cost(double target_speed, int intended_lane, int final_lane,
                          const vector<double>& lane_speeds)
{
    float intended_delta_speed = target_speed - lane_speeds[intended_lane];
    float final_delta_speed = target_speed - lane_speeds[final_lane];

    float cost = (intended_delta_speed + final_delta_speed) / target_speed;
  
    return cost;
}


// Assign a penalty for collision or near collision when attempting to change lanes
float collision_cost(double ego_s, int ego_lane, double ego_speed,
                     int desired_lane, vector<Vehicle> tracked_objects)
{
    float cost = 0.0;
    double forward_distance = FOLLOWING_GAP;
    double rear_distance = CUTIN_GAP;
    // see if any vehicles are within collision range in desired_lane
    for(auto& obj : tracked_objects)
    {
        if(obj.lane == desired_lane)
        {
            double object_s = obj.projected_s;
            double distance = object_s - ego_s;
            if((distance > 0) && (distance < forward_distance))
            {
                forward_distance = distance;
            }
            else if((distance < 0) && (-distance < rear_distance))
            {
                rear_distance = distance;
            }
        }

        if((forward_distance < CAR_LENGTH) || (rear_distance < CAR_LENGTH))
        {
            cost = 1.0;
        }
        else
        {
            float forward_cost = 1.0 - forward_distance / (FOLLOWING_GAP);
            float rear_cost = 1.0 - rear_distance / (CUTIN_GAP);
            cost = (forward_cost + rear_cost) / 2.0;
        }
    }
    return cost;
}

// Assign a penalty for lanes with less free space before an obstructing vehicle
float congestion_cost(double ego_s, int lane,
                       const vector<Vehicle> tracked_objects)
{
    float cost = 0.0;
    double distance = LOOK_AHEAD;
    // find closest object in front of ego in the desired lane
    for(auto& obj : tracked_objects)
    {
        if(obj.lane == lane)
        {
            if((obj.s > ego_s) and (obj.s - ego_s < distance))
            {
                distance = obj.s - ego_s;
            }
        }
        cost = 1 / distance;
    }
    return cost;
}