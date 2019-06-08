#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "tools.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  Tools tools;

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
  // Starting lane
  int lane = 1;

  // Have a reference velocity to target 
  double ref_vel = 0;		//mph
  double max_vel = 49.5;	//mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&tools, &lane, &ref_vel, &max_vel]
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
					
          // log sensor fusion outcome [ id, x, y, vx, vy, s, d] 
          tools.LogSensor(sensor_fusion);

          // current lane center d value 
          double lane_center = 2+lane*4;
					
					
          int prev_size = previous_path_x.size();

          bool too_close = false;
          bool can_switch_left = true;
          bool can_switch_right = true;

          if (prev_size)
            car_s = end_path_s;

          for (int i=0; i<sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            int check_car_lane=-1;
            if (d > 0 && d <=4)             // car on left lane
              check_car_lane = 0;
            if (d > 4 && d <=8)	            // car on middle lane
              check_car_lane = 1;
            if (d > 8 && d <=12)            // car on right lane
              check_car_lane = 2;
            if (check_car_lane < 0)         // car on opposite lanes, ignore
              continue;
						
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // project the vehicle s coordinate to match the timestep of our car reference s 
            check_car_s += (double)prev_size*0.02 * check_speed;

            if (check_car_lane == lane){		// if detected vehicle is on the same lane
              // if the projected vehicle s coordinate is ahead of us and closer than 30m
              if (car_s < check_car_s && (check_car_s-car_s) <30)
                too_close = true;
            }
            else if ((check_car_lane - lane) == -1) {	// if detected vehicle is on the left lane
              if (abs(car_s - check_car_s) < 30){	// if detected vehicle is within +/- 30 m
                if (car_s < check_car_s)		// if detected vehicle is in front 
                  if (check_speed < 20)	//do not switch to this lane if this vehicle is slower than 20 ms-1
                    can_switch_left = false;
                else                // if detected vehicle is behind
                  if (check_speed > 5)	//do not switch to this lane if this vehicle is faster than 5 ms-1
                    can_switch_left = false;
              }
            }
            else if ((check_car_lane - lane) == 1) {	// if detected vehicle is on the right lane
              if (abs(car_s - check_car_s) < 30)
                if (car_s < check_car_s)		// if detected vehicle is in front 
                  if (check_speed < 20)	//do not switch to this lane if this vehicle is slower than 20 ms-1
                    can_switch_right = false;
                else					// if detected vehicle is behind
                  if (check_speed > 5)	//do not switch to this lane if this vehicle is faster than 5 ms-1
                    can_switch_right = false;
            }
          }

          // if one of the other vehicles is too close, deccelerate by reducing ref_vel
          // by steps of 0.447 mph
          // max acceleration or decceleration is 10 ms-2, and each path point execution
          // is 0.02s, thus max change in velocity is 0.2 ms-1, i.e. 0.447 mph
          if (too_close){
            if (lane > 0 && can_switch_left)
              lane--;
            else if (lane != 2 && can_switch_right)
              lane++;
            else
              ref_vel -= 0.447;		//slow down
          }
          else if (ref_vel < max_vel)
            ref_vel += 0.447;
					
          vector<double> ptsx;
          vector<double> ptsy;

          // define 5 anchor points for spline fitting 
          // first two points are previous and current points
          // the other three points are 30m, 60m, 90m away in s from current point (in Frenet coordinate)
          double ref_x;
          double ref_y;
          double ref_x_prev;
          double ref_y_prev;
          double ref_yaw;

          // if not enough left over points from the previous cycle (executed by the controller) 
          // use current car position and orientation
          if (prev_size < 2){
            std::cout<< "prev_size < 2" << std::endl;
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);
            ref_x_prev = ref_x -cos(ref_yaw);
            ref_y_prev = ref_y -sin(ref_yaw);
          }
          // else use the last two points
          else{
            std::cout<< "prev_size >= 2" << std::endl;
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            ref_x_prev = previous_path_x[prev_size-2];
            ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }
          ptsx.emplace_back(ref_x_prev);
          ptsx.emplace_back(ref_x);
          ptsy.emplace_back(ref_y_prev);
          ptsy.emplace_back(ref_y);

          //In Frenet, add evenly 30m spaced points ahead of the starting reference and append to anchor points 
          vector<double> ref_frenet = getFrenet(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y);
          double ref_s = ref_frenet[0];
          double ref_d = ref_frenet[1];
          vector<double> next_wp0 = getXY(ref_s+30,lane_center,map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(ref_s+60,lane_center,map_waypoints_s, map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(ref_s+90,lane_center,map_waypoints_s, map_waypoints_x,map_waypoints_y);

          ptsx.emplace_back(next_wp0[0]);
          ptsx.emplace_back(next_wp1[0]);
          ptsx.emplace_back(next_wp2[0]);

          ptsy.emplace_back(next_wp0[1]);
          ptsy.emplace_back(next_wp1[1]);
          ptsy.emplace_back(next_wp2[1]);

          // Transform to local coordinate frame with starting reference as origin
          for (int i = 0; i<ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
            std::cout<< ptsx[i] << "," << ptsy[i] << std::endl;
          }

          // create a spline with 5 anchor points
          tk::spline s;
          s.set_points(ptsx,ptsy);

          // Define the actual (x,y) points we will use for the planner 
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous leftover path points from last cycle 
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.emplace_back(previous_path_x[i]);
            next_y_vals.emplace_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity 
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with the previous leftover points,
          // here we will always output 50 points
          for (int i=1; i<=50-previous_path_x.size(); i++){
            // The car moves 50 times a second, thus execution period of 0.02sec.
            // ref_vel is in miles/hour, formula to convert to meter/sec => x/2.237
						
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
						
            x_add_on = x_point;

            double x_current = x_point;
            double y_current = y_point;
						
            // Rotate back to world coordinate frame. 
            x_point = x_current*cos(ref_yaw) - y_current*sin(ref_yaw);
            y_point = x_current*sin(ref_yaw) + y_current*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          tools.LogPath(prev_size, car_x, car_y, next_x_vals, next_y_vals);
					
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

