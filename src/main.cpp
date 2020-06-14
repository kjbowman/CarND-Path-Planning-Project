#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <numeric>    // std::accumlate
#include <cfloat>     // FLT_MAX
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include "parameters.h"
#include "vehicle.h"
#include "cost.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // initial car variables
  Vehicle ego;
  // double car_goal_speed = 0;   // target velocity [mph]
  // BehaviorState car_state = BehaviorState::KL;  // state for FSM

  // this huge "lambda" function is the "main loop"
  h.onMessage([&ego, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Ego vehicle's position, pose, and speed "now" (this loop)
          ego.update(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"],
                     j[1]["yaw"], j[1]["speed"]);
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data: list of other cars on the same side of the road
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<Vehicle> tracked_objects;
          for(auto& sf : sensor_fusion) {
            Vehicle obj(sf[SF_ID],
                        sf[SF_X], sf[SF_Y], sf[SF_S], sf[SF_D],
                        0,        // yaw is unknown, assume 0
                        magnitude(sf[SF_VX], sf[SF_VY]));
            obj.projected_s = obj.s;
            tracked_objects.push_back(obj);
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = ego.x;
          double ref_y = ego.y;
          double ref_yaw = deg2rad(ego.yaw);

          int prev_size = previous_path_x.size();

          if(prev_size > 0) {
            ego.projected_s = end_path_s;
            ego.projected_d = end_path_d;
            ego.projected_lane = which_lane(end_path_d);
          
            for(auto& obj : tracked_objects) {
              obj.projected_s += (prev_size * obj.speed / UPDATE_RATE);
            }
          }

          bool too_close = false;

          // check lane speeds, including slower vehicle ahead
          vector<double> lane_speeds(N_LANES);
          for(int lane = 0; lane < N_LANES; ++lane)
          {
            Vehicle target;
            lane_speeds[lane] = ROAD_SPEED_LIMIT;
            if(ego.get_vehicle_ahead(tracked_objects, lane, target)) {
              lane_speeds[lane] = target.speed;
              if(lane == ego.lane) {
                double gap = target.s - ego.projected_s;
                if(gap < FOLLOWING_GAP) {
                  too_close = true;
                }
              }
            }
          }

          // determine best path to plan
          ego.state = ego.choose_next_state(tracked_objects, lane_speeds);

          // std::cout << "State: " << ego.state_string() << std::endl;

          if(too_close)
          {
            ego.slow_down(SPEED_INCREMENT/2);
          }
          else
          {
            ego.speed_up(SPEED_INCREMENT);
          }

          // BEGIN Path Planning

          // if the previous path list is almost empty, use current car position/pose
          if(prev_size < 2) {
            // use 2 pts for path tangent to car angle
            double prev_car_x = ego.x - cos(ego.yaw);
            double prev_car_y = ego.y - sin(ego.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ego.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ego.y);
          }
          else
          {  // use previous path's end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // add some evenly spaced points ahead, in frenet coordinates
          for(int i = 0; i < NUM_SPLINE_PTS; ++i) {
            vector<double> next_wp = getXY(ego.projected_s + (i+1)*SPLINE_PTS_SPACING,
                                          lane_center(ego.final_lane(ego.state)),
                                          map_waypoints_s,
                                          map_waypoints_x,
                                          map_waypoints_y);
            ptsx.push_back(next_wp[0]);
            ptsy.push_back(next_wp[1]);
          }

          for(int i = 0; i < ptsx.size(); ++i) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // create a spline path connecting those points
          tk::spline spline;
          spline.set_points(ptsx, ptsy);

          // new path starting with unused points from previous path
          vector<double> next_x_vals(previous_path_x.begin(), previous_path_x.end());
          vector<double> next_y_vals(previous_path_y.begin(), previous_path_y.end());

          // fill in remaining path points with interpolated spline
          double target_x = SPLINE_PTS_SPACING;
          double target_y = spline(target_x);
          double target_dist = magnitude(target_x, target_y);
          double x_add_on = 0;

          for(int i = 0; i < PATH_BUFFER_SIZE - previous_path_x.size(); ++i) {
            double N = target_dist / mph_to_increment(ego.target_speed);
            double x_point = x_add_on + target_x / N;
            double y_point = spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = ref_x + x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = ref_y + x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // END of path planning - the rest is execution

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}