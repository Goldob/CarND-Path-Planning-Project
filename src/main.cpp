#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::sqrt;
using std::pow;

const int LANE_LEFT = 0;
const int LANE_CENTER = 1;
const int LANE_RIGHT = 2;

const double MIN_VELOCITY = 5.0;
const double MAX_VELOCITY = 20.0;

double LANE_OFFSET_MARGIN = 0.25;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

		  const double timestep = 0.02;

		  static int target_lane = LANE_CENTER;
		  double target_velocity = MAX_VELOCITY;

		  double target_lane_d = 2 + 4 * target_lane;

		  // Process sensor fusion data

		  double MAX_DIST = 1000;
		  double closest_distance_in_front[3] = {MAX_DIST, MAX_DIST, MAX_DIST};
		  double closest_distance_in_back[3] = {MAX_DIST, MAX_DIST, MAX_DIST};

		  for (int i = 0; i < sensor_fusion.size(); i++) {
			  double s = sensor_fusion[i][5];
			  double d = sensor_fusion[i][6];

			  // Discard vehicles driving on the other side of the freeway
			  if (d < 0) continue;

			  double v_x = sensor_fusion[i][3];
			  double v_y = sensor_fusion[i][4];
			  double speed = sqrt(pow(v_x, 2) + pow(v_y, 2));

			  // Predict future car position
			  s += speed * timestep * previous_path_x.size();

			  int lane;
			  if (d < 4) {
				  lane = LANE_LEFT;
			  } else if (d < 8) {
				  lane = LANE_CENTER;
			  } else if (d < 12) {
				  lane = LANE_RIGHT;
			  }

			  double s_diff = s - end_path_s;

			  if (s_diff >= 0 && s_diff < closest_distance_in_front[lane]) {
				  closest_distance_in_front[lane] = s_diff;
			  } else if (s_diff < 0 & -s_diff < closest_distance_in_back[lane]) {
				  closest_distance_in_back[lane] = -s_diff;
			  }
		  }

		  // Make important decisions

		  // Case 1: We are during lane change and haven't reached the target lane yet
		  if (end_path_d < target_lane_d - LANE_OFFSET_MARGIN || end_path_d > target_lane_d + LANE_OFFSET_MARGIN) {
			  // Keep changing lane
		  }

		  // Case 2: It is safe to switch to the right lane
		  else if (target_lane != LANE_RIGHT && closest_distance_in_back [target_lane + 1] > 20
			                                 && closest_distance_in_front[target_lane + 1] > 50) {
			  // Change lane right
			  target_lane += 1;
			  target_velocity = MAX_VELOCITY;
		  }

		  // Case 3: It is safe to stay in our current lane
		  else if (closest_distance_in_front[target_lane] > 20) {
			  // Stay on current lane and drive with maximum velocity
			  target_velocity = MAX_VELOCITY;
		  }

		  // Case 4: It is safe to switch to the left lane
		  else if (target_lane != LANE_LEFT && closest_distance_in_back [target_lane - 1] > 20
			                                && closest_distance_in_front[target_lane - 1] > 30) {
			  // Change lane left
			  target_lane -= 1;
			  target_velocity = MAX_VELOCITY;
		  }

		  // Case 5: There is a vehicle in front of us and we cannot pass it
		  else {
			  // Slow down to keep distance to the vehicle in front
			  target_velocity = MIN_VELOCITY;
		  }

		  vector<double> anchor_x_vals;
		  vector<double> anchor_y_vals;

		  static double current_velocity = 0.0;

		  double ref_x, ref_y, ref_x_prev, ref_y_prev, ref_yaw;
		  int N_prev = previous_path_x.size();
		  if (N_prev >= 2) {
			  ref_x_prev = previous_path_x[N_prev-2];
			  ref_x = previous_path_x[N_prev-1];

			  ref_y_prev = previous_path_y[N_prev-2];
			  ref_y = previous_path_y[N_prev-1];

			  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
		  } else {
			  ref_x_prev = car_x - cos(car_yaw);
			  ref_x = car_x;

			  ref_y_prev = car_y - sin(car_yaw);
			  ref_y = car_y;

			  ref_yaw = deg2rad(car_yaw);
		  }

		  anchor_x_vals.push_back(ref_x_prev);
		  anchor_x_vals.push_back(ref_x);

		  anchor_y_vals.push_back(ref_y_prev);
		  anchor_y_vals.push_back(ref_y);

		  auto ref_frenet = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
		  double anchor_s_inc = 30.0;
		  for (int i = 1; i <= 3; i++) {
			  double s = ref_frenet[0] + i * anchor_s_inc;

			  auto xy = getXY(s, target_lane_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			  anchor_x_vals.push_back(xy[0]);
			  anchor_y_vals.push_back(xy[1]);
		  }

		  // Transform anchor points to the car's coordinate frame
		  for (int i = 0; i < anchor_x_vals.size(); i++) {
			  double x_diff = anchor_x_vals[i] - ref_x;
			  double y_diff = anchor_y_vals[i] - ref_y;

			  anchor_x_vals[i] = x_diff * cos(-ref_yaw) - y_diff * sin(-ref_yaw);
			  anchor_y_vals[i] = x_diff * sin(-ref_yaw) + y_diff * cos(-ref_yaw);
		  }

		  tk::spline s;
		  s.set_points(anchor_x_vals, anchor_y_vals);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

		  // Start with all of the previous path points from last time
		  for (int i = 0; i < previous_path_x.size(); i++) {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }

		  // Calculate how to break up spline points so that we travel at our desired reference velocity
		  double target_x = 30.0;
		  double target_y = s(target_x);
		  double target_d = distance(target_x, target_y, 0, 0);

		  double x_coeff = target_x / target_d;

		  // Generate future path points
		  double x = 0.0;
		  for (int i = 1; i < 50 - previous_path_x.size(); i++)
		  {
			  if (current_velocity < target_velocity) current_velocity += 4.5 * timestep;
			  if (current_velocity > target_velocity) current_velocity -= 4.5 * timestep;

			  x += current_velocity * timestep * x_coeff;
			  double y = s(x);

			  // Transform current point back to the global coordinate frame

			  double global_x = x * cos(ref_yaw) - y * sin(ref_yaw);
			  double global_y = x * sin(ref_yaw) + y * cos(ref_yaw);

			  global_x += ref_x;
			  global_y += ref_y;

			  next_x_vals.push_back(global_x);
			  next_y_vals.push_back(global_y);
		  }

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
