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
  double car_goal_speed = 0;   // target velocity [mph]
  BehaviorState car_state = BehaviorState::KL;  // state for FSM

  // this huge "lambda" function is the "main loop"
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &car_goal_speed, &car_state]
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
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          int car_lane = which_lane(car_d);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          vector<TrackedObject> tracked_objects;
          for(auto& object : sensor_fusion) {
            TrackedObject obj;
            obj.id = object[SF_ID];
            obj.x = object[SF_X];
            obj.y = object[SF_Y];
            obj.vx = object[SF_VX];
            obj.vy = object[SF_VY];
            obj.s = object[SF_S];
            obj.d = object[SF_D];
            tracked_objects.push_back(obj);
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          int prev_size = previous_path_x.size();

          if(prev_size > 0) {
            car_s = end_path_s;
            car_lane = which_lane(end_path_d);
          }

          bool too_close = false;

          // check cars around us
          vector<vector<double>> object_speeds(N_LANES);
          
          for(auto& obj : tracked_objects) {
            // compute average speed of each lane from detected targets
            if(obj.d < 0)
              continue;
            // we only care about objects directly around us (next to and ahead)
            // object distance, projected out in time to match end of our path
            double object_speed = obj.speed();
            double object_s = obj.projected_s(prev_size);
            if((object_s > car_s - CUTIN_GAP) /* && (object_s < car_s + 75.0) */) {
              int object_lane = obj.lane();
              object_speeds[object_lane].push_back(object_speed);
              if((object_lane == car_lane)
                  && (object_s > car_s)
                  && (object_s - car_s < FOLLOWING_GAP)) {
                too_close = true;
              }
            }
          }

          // compute average speed of all detected object vehicles in each lane
          vector<double> lane_speeds(N_LANES);
          for(int lane = 0; lane < N_LANES; lane++) {
            if(object_speeds[lane].empty()) {
              lane_speeds[lane] = SPEED_LIMIT;
            } else {
              // compute average speed from all objects in lane
              lane_speeds[lane] = std::accumulate(object_speeds[lane].begin(),
                                                 object_speeds[lane].end(), 0)
                                 / object_speeds[lane].size();
            }
          }
          
          // Control acceleration (smooth transition between set speeds)
          if(too_close) {   // slow down
            car_goal_speed -= SPEED_INCREMENT;
          } else if(car_goal_speed < SPEED_LIMIT) {
            car_goal_speed += SPEED_INCREMENT;  // speed up to speed limit
          }

          // determine best path to plan
          vector<BehaviorState> next_states = successor_states(car_state, car_lane);
          float best_cost = FLT_MAX;
          BehaviorState best_state = car_state;
          for(auto& state : next_states) {
            // cost based on lane speeds
            float speed_cost = inefficiency_cost(SPEED_LIMIT,
                                                car_lane + intended_lane(state),
                                                car_lane + final_lane(state), 
                                                lane_speeds);
            // cost based on collision risk
            float crash_cost = collision_cost(car_s, car_lane, car_speed,
                                              car_lane + final_lane(state),
                                              tracked_objects, prev_size);
            // cost based on (inverse of) free space in desired final lane
            float crowding_cost = congestion_cost(car_s,
                                                  car_lane + final_lane(state),
                                                  tracked_objects);
            float cost = speed_cost * pow(10, 6)
                       + crash_cost * pow(10, 8)
                       + crowding_cost * pow(10, 5);
            
            if(cost < best_cost) {
              best_cost = cost;
              best_state = state;
            }
          }
          // std::cout << "best cost: " << best_cost
          //           << "\t" << print_state(car_state) << "\t->\t" << print_state(best_state)
          //           << std::endl;
          car_state = best_state;

          // BEGIN Path Planning

          // if the previous path list is almost empty, use current car position/pose
          if(prev_size < 2) {
            // use 2 pts for path tangent to car angle
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
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
            vector<double> next_wp = getXY(car_s + (i+1)*SPLINE_PTS_SPACING,
                                          lane_center(car_lane + final_lane(car_state)),
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
            double N = target_dist / mph_to_increment(car_goal_speed);
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