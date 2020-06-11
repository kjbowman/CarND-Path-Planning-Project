#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;


// simulator/model parameters
const unsigned N_LANES = 3;
const unsigned LEFT_LANE = 0;           // midside lane
const unsigned RIGHT_LANE = N_LANES-1;  // outside lane
const double LANE_WIDTH = 4.0;          // [m]
const double UPDATE_RATE = 50;          // [Hz]

// conversion constants
const double METERS_PER_MILE = 1609.3;
const double SECONDS_PER_HOUR = 3600;

// project constraints
const double ROAD_SPEED_LIMIT = 50.0;
const double MAX_ACCEL = 10.0;          // [m/s^2]
const double MAX_JERK = 10.0;           // [m/s^3]
const double MAX_LANE_CHANGE_TIME = 3.0;  // [s]

// constants for tuning the algorithm
const int PATH_BUFFER_SIZE = 30;  // 50
const int NUM_SPLINE_PTS = 4;
const double SPLINE_PTS_SPACING = 30.0;   // [m]
const double CAR_LENGTH = 5.0;            // [m] (a guess based on avg car length)
const double FOLLOWING_GAP = CAR_LENGTH * 3;        // [m]
const double CUTIN_GAP = CAR_LENGTH * 2;            // [m]
const double SPEED_LIMIT = ROAD_SPEED_LIMIT - 0.5;  // [mph]
const double SPEED_INCREMENT = 0.25;      // [mph/update_rate]

// Sensor fusion list indexes
const int SF_ID = 0;
const int SF_X = 1;
const int SF_Y = 2;
const int SF_VX = 3;
const int SF_VY = 4;
const int SF_S = 5;
const int SF_D = 6;

enum class BehaviorState { KL, LCL, LCR, PLCL, PLCR };

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
// compute the magnitude of a resultant vector
double magnitude(double x, double y) { return sqrt(x*x + y*y); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

// convert speed in mph to a distance increment, based on 50Hz update rate
double mph_to_increment(double mph) {
  return mph * (METERS_PER_MILE/SECONDS_PER_HOUR) / UPDATE_RATE;
}

// get the d-coordinate of the center of lane for a given lane
double lane_center(int lane) {
  return LANE_WIDTH * ((double)lane + 0.5);
}

int which_lane(double d) {
  return (int)(d / LANE_WIDTH);
}

class TrackedObject {
public:
  int id;
  double x, y;
  double vx, vy;
  double s, d;
  
  TrackedObject() : id(-1), x(0.0), y(0.0), vx(0.0), vy(0.0), s(0.0), d(0.0) {};
  ~TrackedObject() {};

  double speed() const { return magnitude(vx, vy); }
  int lane() const { return which_lane(d); }
  double projected_s(int n_samples) {
    return  s + (n_samples * speed() / UPDATE_RATE);
  }
};

int intended_lane(BehaviorState state) {
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

  return lane;
}

int final_lane(BehaviorState state) {
  int lane = 0;
  switch(state) {
    case BehaviorState::LCL:
    lane = -1;
    break;

    case BehaviorState::LCR:
    lane = 1;
    break;
  }

  return lane;
}

// adapted from "Implement a Second Cost Function in C++" lesson
// target_speed: desired speed (i.e. near speed limit)
// intended_lane: intendend lane for the given behavior (PLCL/R, LCL/R => one lane over) 
// final_lane: immediate resulting lane of the behavior. LCL/R => one lane over
// lane_speeds: vector of lane speeds , based on sensed traffic
float inefficiency_cost(double target_speed, int intended_lane, int final_lane,
                          const vector<double>& lane_speeds) {
  float intended_delta_speed = target_speed - lane_speeds[intended_lane];
  float final_delta_speed = target_speed - lane_speeds[final_lane];

  // float cost = 1 - exp(-fabs(intended_delta_speed + final_delta_speed));
  float cost = (intended_delta_speed + final_delta_speed) / target_speed;
  
  return cost;
}

float collision_cost(double ego_s, int ego_lane, double ego_speed, int desired_lane,
                     vector<TrackedObject> tracked_objects, int path_size) {
  float cost = 0.0;
  double forward_distance = FOLLOWING_GAP;
  double rear_distance = CUTIN_GAP;
  // see if any vehicles are within collision range in desired_lane
  for(auto& obj : tracked_objects) {
    if(obj.lane() == desired_lane) {
      double object_s = obj.projected_s(path_size);
      double distance = object_s - ego_s;
      if((distance > 0) && (distance < forward_distance)) {
        forward_distance = distance;
      } else if((distance < 0) && (-distance < rear_distance)) {
        rear_distance = distance;
      }
    }

    if((forward_distance < CAR_LENGTH) || (rear_distance < CAR_LENGTH)) {
      cost = 1.0;
    } else {
      float forward_cost = 1.0 - forward_distance / (FOLLOWING_GAP);
      float rear_cost = 1.0 - rear_distance / (CUTIN_GAP);
      cost = (forward_cost + rear_cost) / 2.0;
    }
  }

  return cost;
}

float congestion_cost(double ego_s, int lane,
                       const vector<TrackedObject> tracked_objects) {
  float cost = 0.0;
  double distance = 1000;
  // find closest object in front of ego in ego's lane and in the desired lane
  for(auto& obj : tracked_objects) {
    if(obj.lane() == lane) {
      if((obj.s > ego_s) and (obj.s - ego_s < distance)) {
        distance = obj.s - ego_s;
      }
    }
  
    cost = 1 / distance;
  }

  return cost;
}

vector<BehaviorState> successor_states(BehaviorState current_state, int lane) {
  vector<BehaviorState> next_states {BehaviorState::KL};  // KL is always possible next state

  switch(current_state) {
    case BehaviorState::KL:
    // next_states.push_back(BehaviorState::PLCL);
    // next_states.push_back(BehaviorState::PLCR);
    if(lane != LEFT_LANE) {
      next_states.push_back(BehaviorState::LCL);
    }
    if(lane != RIGHT_LANE) {
      next_states.push_back(BehaviorState::LCR);
    }
    break;

    case BehaviorState::LCL:
    if(lane != LEFT_LANE) {
      next_states.push_back(BehaviorState::LCL);
    }
    break;

    case BehaviorState::LCR:
    if(lane != RIGHT_LANE) {
      next_states.push_back(BehaviorState::LCR);
    }
    break;

    case BehaviorState::PLCL:
    if(lane != LEFT_LANE) {
      next_states.push_back(BehaviorState::PLCL);
      next_states.push_back(BehaviorState::LCL);
    }
    break;

    case BehaviorState::PLCR:
    if(lane != RIGHT_LANE) {
      next_states.push_back(BehaviorState::PLCR);
      next_states.push_back(BehaviorState::LCR);
    }
    break;

    default:
    break;
  }

  return next_states;
}

string print_state(BehaviorState state) {
  string out = "?";
  switch(state) {
    case BehaviorState::KL:   out = "KL";   break;
    case BehaviorState::PLCL: out = "PLCL"; break;
    case BehaviorState::PLCR: out = "PLCR"; break;
    case BehaviorState::LCL:  out = "LCL";  break;
    case BehaviorState::LCR:  out = "LCR";  break;
  }
  return out;
}

#endif  // HELPERS_H