#ifndef CAR_H
#define CAR_H
struct car
{
   double _x;
   double _y;
   double _s;
   double _d;
   double _yaw;
   int _lane;
   double _speed;
   double _distance = 0;
  car(double x,double y,double s,double d,double yaw,int lane,double speed):_x(x),_y(y),_s(s),
_yaw(yaw),_lane(lane),_speed(speed){}; 
};
#endif
