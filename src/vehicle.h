#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <vector>
#include <string>

using std::map;
using std::vector;
using std::string;

enum class BehaviorState
{
    KL,     // Keep Lane
    LCL,    // Lane Change Left
    LCR,    // Lane Change Right
    PLCL,   // Prepare Lane Change Left
    PLCR    // Prepare Lane Change Right
};

class Vehicle
{
public:
    // constructors
    Vehicle();
    Vehicle(int id, double x, double y, double s, double d,
            double yaw, double speed);
    
    // destructor
    virtual ~Vehicle() {};

    // methods
    void update(double x, double y, double s, double d,
                double yaw, double speed);

    void reset();
    
    BehaviorState choose_next_state(vector<Vehicle>& tracked_objects,
                                    vector<double> lane_speeds);
    
    vector<BehaviorState> successor_states();

    bool get_vehicle_ahead(vector<Vehicle>& tracked_objects, int lane,
                           Vehicle& rVehicle);
    
    bool get_vehicle_behind(vector<Vehicle>& tracked_objects, int lane,
                            Vehicle& rVehicle);

    void slow_down(double increment);

    void speed_up(double increment);

    int intended_lane(BehaviorState state);

    int final_lane(BehaviorState state);

    string state_string();

    map<BehaviorState, int> lane_direction = 
    {
        {BehaviorState::PLCL, 1},
        {BehaviorState::LCL, 1},
        {BehaviorState::LCR, -1},
        {BehaviorState::PLCR, -1}
    };

    // member attributes
    int id;
    double x, y;
    double s, d;
    double yaw;
    double speed;
    int lane;
    BehaviorState state;

    double target_speed;

    double projected_s, projected_d;
    double projected_lane;

};

#endif  // VEHICLE_H
