#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "BehaviourPlanner.h"
#include "TrajectoryPlanner.h"
<<<<<<< HEAD
#include "HighwayMap.h"
=======
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() 
{

    uWS::Hub h;

<<<<<<< HEAD
    // Waypoint map to read from
    // The max s value before wrapping around the track back to 0

    HighwayMap* hMap = HighwayMap::getInstance();
    BehaviourPlanner b(hMap);
    TrajectoryPlanner t(hMap);
    car carCurr(&b, &t);


     h.onMessage([&b, &t, &carCurr, hMap]
=======
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x{};
    vector<double> map_waypoints_y{};
    vector<double> map_waypoints_s{};
    vector<double> map_waypoints_dx{};
    vector<double> map_waypoints_dy{};

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  
   
    string line;
    while (getline(in_map_, line)) 
    {
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



     h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx,&map_waypoints_dy]
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) 
         {
             // "42" at the start of the message means there's a websocket message event.
             // The 4 signifies a websocket message
             // The 2 signifies a websocket event
             if (length && length > 2 && data[0] == '4' && data[1] == '2') {

                 auto s = hasData(data);

                 if (s != "") 
                 {
                     auto j = json::parse(s);
                     string event = j[0].get<string>();

                        if (event == "telemetry") {
                            // j[1] is the data JSON object

                            // Main car's localization Data
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
<<<<<<< HEAD
                            
=======
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285

                            // Sensor Fusion Data, a list of all other cars on the same side 
                            //   of the road.
                            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


<<<<<<< HEAD
=======
                            
                           

>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
                            /**
                             * TODO: define a path made up of (x,y) points that the car will visit
                             *   sequentially every .02 seconds
                            */

                            //START ----------------------
<<<<<<< HEAD
                            carCurr.setValues(car_x, car_y, car_s, car_d, deg2rad(car_yaw), (car_d / 4), (car_speed / 2.24), end_path_s, end_path_d);
                            
                            
                            carCurr._bp->setEnvironment(carCurr, previous_path_x.size(), sensor_fusion);
                            pair<double, int> next =  carCurr._bp->returnNextAction();
                           
                            
                            vector<vector<double>> nextTraj = carCurr._tp->generateTrajectory
                            (previous_path_x, previous_path_y, 
                                carCurr, 
                                next.first, next.second);
                            
=======
                            cout << endl << "New main Call" << endl;
                            car carCurr(car_x, car_y, car_s, car_d, car_yaw, (car_d / 4), car_speed);
                            
                            std::cout << " Car values: " << carCurr._s << "," << carCurr._d << "," << carCurr._speed << "," << carCurr._lane << "," << std::endl;


                            cout << endl << "Created behaviour planner" << endl;
                            BehaviourPlanner b(carCurr, previous_path_x.size(), sensor_fusion);
                            pair<double, int> next = b.returnNextAction();

                            cout << endl << "got next action" << endl;
                            cout << next.first << "," << next.second << endl;
                            
                            cout << endl << "Creating trajectory planner" << endl;
                            TrajectoryPlanner t;  
                            vector<vector<double>> nextTraj = t.generateTrajectory
                            (previous_path_x, previous_path_y, 
                                carCurr, 
                            map_waypoints_x, map_waypoints_y, 
                            map_waypoints_s, next.first, next.second);
                            cout << endl << "got trajectory " << endl;
                            
                            /*
                            cout << "x size: " << nextTraj[0].size() << ", y size: " << nextTraj[1].size() << endl;
                            cout << nextTraj[0].back() << "," << nextTraj[1].back() << endl;
                            */
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
                            
                            //END ------------------------
                            json msgJson;

<<<<<<< HEAD
                           
=======
>>>>>>> 438f8d990020bfd1fcec447adb2653dbd9116285
                            msgJson["next_x"] = nextTraj[0];
                            msgJson["next_y"] = nextTraj[1];

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }  // end "telemetry" if
                    }
                    else {
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
        char* message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}



