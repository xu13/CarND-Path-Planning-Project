#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "planner.h"
#include "helper.h"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



int main() {
  uWS::Hub h;

  // Load map
  std::string map_file_ = "../data/highway_map.csv";
  Map map(map_file_);
  Car car(0);
  Planner planner(&map, &car);

  h.onMessage([&map, &car, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
            double car_x = j[1]["x"].get<double>();
            double car_y = j[1]["y"].get<double>();
            double car_s = j[1]["s"].get<double>();
            double car_d = j[1]["d"].get<double>();
            double car_yaw = deg2rad(j[1]["yaw"].get<double>());
            double car_speed = mph2mps(j[1]["speed"].get<double>());

          	// Previous path data given to the Planner
            std::vector<double> previous_path_x = j[1]["previous_path_x"].get<std::vector<double>>();
            std::vector<double> previous_path_y = j[1]["previous_path_y"].get<std::vector<double>>();
          	// Previous path's end s and d values 
            double end_path_s = j[1]["end_path_s"].get<double>();
            double end_path_d = j[1]["end_path_d"].get<double>();

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"].get<std::vector<std::vector<double> >>();

          	json msgJson;

          	std::vector<double> next_x_vals;
          	std::vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // Update car states
            car.update(car_x, car_y, car_s, car_d, car_yaw, car_speed);

            // Make objects
            std::unordered_map<int, Object> objects;
            for (auto& fusion_data : sensor_fusion) {
              int id = static_cast<int>(fusion_data[0]);
              double x = fusion_data[1];
              double y = fusion_data[2];
              double vx = fusion_data[3];
              double vy = fusion_data[4];
              // s/d are not accurate due to the coarse map representation
              double s = fusion_data[5];
              double d = fusion_data[6];

              std::vector<double> vsvd = map.getVelocityFrenet(vx, vy, s);
              Object obj(id, x, y, vx, vy, s, d, vsvd[0], vsvd[1]);
              objects.insert({id, obj});
            }

            // Previous waypoints left
            size_t prev_size = previous_path_x.size();

            // Generate plan
            planner.plan(objects, prev_size, &next_x_vals, &next_y_vals);

						// END TODO

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
