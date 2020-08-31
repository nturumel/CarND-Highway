#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <algorithm>  
#include <vector>
#include "car.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;

double max_acc=0.448;
double lane_width=4;

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
vector<double> getXY(double s, double d, 
                     const vector<double> &maps_s,
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) 
{
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
void generateTrajectory(
  vector<double> previous_path_x, 
  vector<double> previous_path_y,
  vector<double>& next_x_vals, 
  vector<double>& next_y_vals, 
  car& carCurr,
  vector<double> map_waypoints_x, 
  vector<double> map_waypoints_y, 
  vector<double> map_waypoints_s,
  double ref_vel, int finalLane, int size = 20)
{
  
  double& car_vel = carCurr._speed; 
  double ref_x = carCurr._x;
  double ref_y = carCurr._y;
  double ref_yaw = deg2rad(carCurr._yaw);
  int& lane = finalLane;
  
  
  double vel_add=0;
  double delta=fabs(ref_vel-car_vel);
  int steps=1;

  if(ref_vel < car_vel)
  {
  	vel_add=(-1*std::min<double>((delta/steps),max_acc));
  }
  else if(ref_vel > car_vel)
  {
  	vel_add=std::min<double>((delta/steps),max_acc);
  }
  if (carCurr._lane != lane)
  {
      vel_add *= 0.8;
  }
  int previous_path_size = previous_path_x.size();
  vector<double> ptsx{};
  vector<double> ptsy{};

  std::cout << " Car values: " << car_vel << "," << lane << "," << vel_add << std::endl;
  
  if(previous_path_size < 2)
  {
    double prev_car_x = carCurr._x-cos(carCurr._yaw); 
    double prev_car_y = carCurr._y-sin(carCurr._yaw); 

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
  }
  else
  {
    ref_x=previous_path_x[previous_path_size-1];
    ref_y=previous_path_y[previous_path_size-1];


    double prev_ref_x=previous_path_x[previous_path_size-2];
    double prev_ref_y=previous_path_y[previous_path_size-2];
    ref_yaw=atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);

    //Use two points to tangent
    ptsx.push_back( prev_ref_x);
    ptsx.push_back( ref_x);

    ptsy.push_back( prev_ref_y);
    ptsy.push_back( ref_y);
  }

  
  //In Frenet
  vector<double> next_wp0= getXY((carCurr._s+30),(lane_width*(lane)+lane_width/2)
  ,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1= getXY((carCurr._s+60),(lane_width*(lane)+lane_width/2),
  map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2= getXY((carCurr._s+90),(lane_width*(lane)+lane_width/2),
  map_waypoints_s,map_waypoints_x,map_waypoints_y);

  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  //transform to car coordinates
  for(int i=0;i<ptsx.size();++i)
  {
    double shift_x=ptsx[i]-ref_x;
    double shift_y=ptsy[i]-ref_y;
    
    ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }
  
  //create a spline
  tk::spline s;
  s.set_points(ptsx,ptsy);
 
  
  //we are pushing back all the pts left from our previous cycle
  for(int i=0;i<previous_path_size;++i)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  } 
  
  
  //we now select points on the spline
  double target_x=30;
  double target_y=s(target_x);
  double d=sqrt(target_x*target_x+target_y*target_y);
  
  double add_on=0;
  (car_vel)+=vel_add;
  
  int N = d/(0.02*(car_vel)/2.24);
    
    
  while(next_x_vals.size()<size)
  {
    double add=(target_x)/N;
    double x_point=add_on+add;
    double y_point=s(x_point);
    
    add_on=x_point;
    
    //transform to global
    double x_ref=x_point; 
    double y_ref=y_point;
    
    x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
    
    x_point +=ref_x;
    y_point +=ref_y;
    
    // push into next vec
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
    
    carCurr._x+=add;    
  }
  
  //check if end lane reached if not keep adding .. 
  bool hitLane = true;
  int after = 0;
  while (hitLane)
  {
      double backX = next_x_vals[next_x_vals.size() - 1];
      double backY = next_y_vals[next_y_vals.size() - 1];

      double prevBackX = next_x_vals[next_x_vals.size() - 2];
      double prevBackY = next_y_vals[next_y_vals.size() - 2];

      double theta = atan2(backY - prevBackY, backX - prevBackX);
      theta = rad2deg(theta);

      vector<double> frenet = getFrenet(backX, backY, theta, map_waypoints_x, map_waypoints_y);

      int lane = frenet[1] / lane_width; // this is not working 
      std::cout << "theta: " << theta << std::endl;
      if (lane == finalLane)
      {
          hitLane = false;
          after++;
      }
      else
      {
          double add = (target_x) / N;
          double x_point = add_on + add;
          double y_point = s(x_point);

          add_on = x_point;

          //transform to global
          double x_ref = x_point;
          double y_ref = y_point;

          x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
          y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

          x_point += ref_x;
          y_point += ref_y;

          // push into next vec
          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);

          carCurr._x += add;
      }
  }

  return;
  
}
#endif  // HELPERS_H
