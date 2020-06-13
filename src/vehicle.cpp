#include <math.h>
#include <cfloat>   // FLT_MAX
#include "parameters.h"
#include "vehicle.h"
#include "cost.h"
#include "helpers.h"

Vehicle::Vehicle()
{
    id = -1;    // default: -1 indicates "ego" vehicle
    state =  BehaviorState::KL;
    target_speed = 0;
}

Vehicle::Vehicle(int id, double x, double y, double s, double d,
                 double yaw, double speed)
{
    this->id = id;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
    this->target_speed = 0;

    lane = (d < 0) ? -1 : which_lane(d);
    state = BehaviorState::KL;
    target_speed = 0;
}

void Vehicle::update(double x, double y, double s, double d, double yaw, double speed)
{
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;

    this->lane = (d < 0) ? -1 : which_lane(d);

    this->projected_s = s;
    this->projected_d = d;
    this->projected_lane = this->lane;
}

BehaviorState Vehicle::choose_next_state(vector<Vehicle>& tracked_objects,
                                         vector<double> lane_speeds)
{
    vector<BehaviorState> states = successor_states();
    float best_cost = FLT_MAX;
    BehaviorState best_state = this->state;

    for(auto& s: states)
    {
        // cost based on lane speeds
        float speed_cost = inefficiency_cost(ROAD_SPEED_LIMIT,
                                             intended_lane(s),
                                             final_lane(s),
                                             lane_speeds);
        // cost based on collision risk
        float crash_cost = collision_cost(this->projected_s, this->lane, this->speed,
                                          final_lane(s),
                                          tracked_objects);
        // cost based on (inverse of) free space in desired final lane
        float crowding_cost = congestion_cost(this->projected_s,
                                              final_lane(s),
                                              tracked_objects);
        float cost = speed_cost * pow(10, 5)
                    + crash_cost * pow(10, 8)
                    + crowding_cost * pow(10, 6);
        
        if(cost < best_cost) {
            best_cost = cost;
            best_state = s;
        }
    }
    return best_state;
}

vector<BehaviorState> Vehicle::successor_states()
{
    // start the list with KL, which is always a possible next state
    vector<BehaviorState> next_states {BehaviorState::KL};

    switch(this->state) {
        case BehaviorState::KL:
        // next_states.push_back(BehaviorState::PLCL);
        // next_states.push_back(BehaviorState::PLCR);
        if(this->lane != LEFT_LANE) {
            next_states.push_back(BehaviorState::LCL);
        }
        if(this->lane != RIGHT_LANE) {
            next_states.push_back(BehaviorState::LCR);
        }
        break;

        case BehaviorState::LCL:
        if(this->lane != LEFT_LANE) {
            next_states.push_back(BehaviorState::LCL);
        }
        break;

        case BehaviorState::LCR:
        if(this->lane != RIGHT_LANE) {
            next_states.push_back(BehaviorState::LCR);
        }
        break;

        case BehaviorState::PLCL:
        if(this->lane != LEFT_LANE) {
            next_states.push_back(BehaviorState::PLCL);
            next_states.push_back(BehaviorState::LCL);
        }
        break;

        case BehaviorState::PLCR:
        if(this->lane != RIGHT_LANE) {
            next_states.push_back(BehaviorState::PLCR);
            next_states.push_back(BehaviorState::LCR);
        }
        break;

        default:    // unreachable
        break;
    }

    return next_states;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle>& predictions, int lane,
                                Vehicle& rVehicle)
{
    bool found = false;
    double min_distance = FLT_MAX;
    for(auto& obj : predictions)
    {
        if(obj.lane == lane && obj.s > this->s)
        {
            double distance = obj.s - this->s;
            if(distance < min_distance)
            {
                found = true;
                min_distance = distance;
                rVehicle = obj;
            }
        }
    }
    return found;
}

bool Vehicle::get_vehicle_behind(vector<Vehicle>& predictions, int lane,
                                 Vehicle& rVehicle)
{
    bool found = false;
    double min_distance = FLT_MAX;
    for(auto& obj : predictions)
    {
        if(obj.lane == lane && obj.s < this->s)
        {
            double distance = this->s - obj.s;
            if(distance < min_distance)
            {
                found = true;
                min_distance = distance;
                rVehicle = obj;
            }
        }
    }
    return found;

}

void Vehicle::slow_down()
{
    target_speed -= SPEED_INCREMENT;
}

void Vehicle::speed_up()
{
    if(target_speed < SPEED_LIMIT)
    {
        target_speed += SPEED_INCREMENT;
    }
}

int Vehicle::intended_lane(BehaviorState state) {
  int lane = 0;
  switch(state) {
    case BehaviorState::PLCL:
    case BehaviorState::LCL:
    lane = -1;
    break;

    case BehaviorState::PLCR:
    case BehaviorState::LCR:
    lane = 1;
    break;
  }

  return this->lane + lane;
}

int Vehicle::final_lane(BehaviorState state) {
  int lane = 0;
  switch(state) {
    case BehaviorState::LCL:
    lane = -1;
    break;

    case BehaviorState::LCR:
    lane = 1;
    break;
  }

  return this->lane + lane;
}
