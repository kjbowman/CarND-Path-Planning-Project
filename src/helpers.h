#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "parameters.h"

// for convenience
using std::string;
using std::vector;


string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);
// compute the magnitude of a resultant vector
double magnitude(double x, double y);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);;

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y);;

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y);

// convert speed in mph to a distance increment, based on 50Hz update rate
double mph_to_increment(double mph);

// get the d-coordinate of the center of lane for a given lane
double lane_center(int lane);

int which_lane(double d);

/* 
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
 */
#endif  // HELPERS_H